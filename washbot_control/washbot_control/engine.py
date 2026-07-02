"""Mission engine: the plan -> execute -> monitor -> replan loop, ROS-free.

The engine owns *decisions*; it never talks to hardware. The mission
controller node feeds it step outcomes and it answers with the next step to
dispatch, a replanning event, or a terminal state. Keeping this logic out of
the ROS node means the entire recovery behaviour — blocked passages, retry
budgets, replan limits, unreachable goals — is covered by fast unit tests.

Recovery policy:

- ``move`` fails    -> the traversed edge is blocked in the world model and
                       the mission replans from the robot's current location.
- ``clean`` fails   -> retried in place up to ``max_step_retries`` times;
                       a fixture that still fails aborts the mission (a broken
                       actuator cannot be fixed by planning around it).
- replanning        -> bounded by ``max_replans``; hitting the bound or an
                       unsolvable problem aborts with a clear reason.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence

from washbot_planning.backends import PlanningResult
from washbot_planning.pddl_core import parse_domain, parse_problem, validate_plan
from washbot_planning.pddl_core.model import PddlError
from washbot_planning.plan import Plan, PlanStep
from washbot_planning.problem_builder import (
    build_strips_problem,
    build_temporal_problem,
)

from washbot_control.world_model import WorldModel

PlanFn = Callable[[str, str], PlanningResult]


class MissionState(enum.Enum):
    IDLE = 'idle'
    PLANNING = 'planning'
    EXECUTING = 'executing'
    SUCCEEDED = 'succeeded'
    ABORTED = 'aborted'


@dataclass
class MissionEvent:
    kind: str  # 'plan', 'replan', 'step_ok', 'step_failed', 'retry', 'abort', 'success'
    detail: str


@dataclass
class MissionEngine:
    world: WorldModel
    goals: List[str]
    domain_text: str
    plan_fn: PlanFn
    domain_mode: str = 'strips'  # 'strips' | 'temporal'
    max_replans: int = 5
    max_step_retries: int = 1
    validate_plans: bool = True

    state: MissionState = MissionState.IDLE
    plan: Optional[Plan] = None
    step_index: int = 0
    replans: int = 0
    abort_reason: str = ''
    events: List[MissionEvent] = field(default_factory=list)
    _retries_left: int = 0

    # ----------------------------------------------------------------- start

    def start(self) -> bool:
        """Compute the initial plan. Returns True when execution can begin."""
        unknown = [g for g in self.goals if g not in self.world.locations]
        if unknown:
            return self._abort(f'unknown goal locations: {", ".join(unknown)}')
        already_clean = self.world.goals_satisfied(self.goals)
        if already_clean:
            self.state = MissionState.SUCCEEDED
            self._log('success', 'all goals already satisfied')
            return False
        return self._plan(initial=True)

    # ------------------------------------------------------------ step cycle

    def current_step(self) -> Optional[PlanStep]:
        if self.state is not MissionState.EXECUTING or self.plan is None:
            return None
        if self.step_index >= len(self.plan.steps):
            return None
        return self.plan.steps[self.step_index]

    def report_step_result(self, success: bool, message: str = '') -> None:
        """Feed the outcome of the currently dispatched step back in."""
        step = self.current_step()
        if step is None:
            raise RuntimeError('report_step_result called with no active step')

        if success:
            self._log('step_ok', step.signature)
            self._apply_effects(step)
            self.step_index += 1
            self._retries_left = self.max_step_retries
            if self.step_index >= len(self.plan.steps):
                self._finish()
            return

        self._log('step_failed', f'{step.signature}: {message or "no detail"}')
        if step.name == 'move':
            origin, destination = step.args[-2], step.args[-1]
            self.world.block_edge(origin, destination)
            self._log('replan', f'blocked edge {origin} -> {destination}')
            self._plan()
        elif self._retries_left > 0:
            self._retries_left -= 1
            self._log('retry', f'{step.signature} '
                               f'({self._retries_left} retries left)')
            # The step stays current; the controller dispatches it again.
        else:
            self._abort(f'step {step.signature} failed with no retries left: '
                        f'{message or "no detail"}')

    # ------------------------------------------------------------- internals

    def _apply_effects(self, step: PlanStep) -> None:
        if step.name == 'move':
            self.world.apply_move(step.args[-1])
        elif step.name in ('clean', 'deep_clean'):
            self.world.apply_clean(step.args[-1])
        elif step.name == 'recharge':
            self.world.recharge()

    def _finish(self) -> None:
        if self.world.goals_satisfied(self.goals):
            self.state = MissionState.SUCCEEDED
            self._log('success', f'all {len(self.goals)} goals cleaned')
        else:
            # Plan exhausted but goals unmet (should not happen with a valid
            # plan); replan rather than silently succeeding.
            self._log('replan', 'plan exhausted with unmet goals')
            self._plan()

    def _plan(self, initial: bool = False) -> bool:
        if not initial:
            if self.replans >= self.max_replans:
                return self._abort(f'replan limit reached ({self.max_replans})')
            self.replans += 1

        self.state = MissionState.PLANNING
        snapshot = self.world.snapshot()
        remaining = [g for g in self.goals if g not in self.world.cleaned]

        if self.domain_mode == 'temporal':
            problem_text = build_temporal_problem(snapshot, remaining)
        else:
            # In STRIPS mode there is no deep_clean action; treat deep-dirty
            # fixtures as ordinarily dirty so they stay reachable goals.
            snapshot.dirty = sorted(set(snapshot.dirty) | set(snapshot.deep_dirty))
            snapshot.deep_dirty = []
            problem_text = build_strips_problem(snapshot, remaining)

        result = self.plan_fn(self.domain_text, problem_text)
        if not result.success or result.plan is None or not result.plan.steps:
            return self._abort(f'planning failed ({result.backend}): {result.error}')

        if self.validate_plans and self.domain_mode == 'strips':
            try:
                report = validate_plan(parse_domain(self.domain_text),
                                       parse_problem(problem_text),
                                       result.plan.as_tuples())
                if not report.valid:
                    return self._abort(f'planner returned an invalid plan: '
                                       f'{report.summary()}')
            except PddlError as error:
                return self._abort(f'plan validation error: {error}')

        self.plan = result.plan
        self.step_index = 0
        self._retries_left = self.max_step_retries
        self.state = MissionState.EXECUTING
        kind = 'plan' if initial else 'replan'
        self._log(kind, f'{len(result.plan)} steps via {result.backend}: '
                        + ' '.join(s.signature for s in result.plan.steps))
        return True

    def _abort(self, reason: str) -> bool:
        self.state = MissionState.ABORTED
        self.abort_reason = reason
        self._log('abort', reason)
        return False

    def _log(self, kind: str, detail: str) -> None:
        self.events.append(MissionEvent(kind=kind, detail=detail))

    # ---------------------------------------------------------------- report

    def summary(self) -> dict:
        return {
            'state': self.state.value,
            'goals': list(self.goals),
            'cleaned': sorted(self.world.cleaned),
            'robot_at': self.world.robot_at,
            'battery_pct': round(self.world.battery_pct, 1),
            'plan_steps': len(self.plan.steps) if self.plan else 0,
            'steps_executed': self.step_index,
            'replans': self.replans,
            'blocked_edges': sorted(f'{a}->{b}' for a, b in self.world.blocked),
            'abort_reason': self.abort_reason,
            'events': [{'kind': e.kind, 'detail': e.detail} for e in self.events],
        }


def make_local_plan_fn(backend_name: str = 'auto',
                       timeout_seconds: float = 60.0) -> PlanFn:
    """Plan through a local backend (internal search or POPF subprocess)."""
    from washbot_planning import backends

    def plan_fn(domain_text: str, problem_text: str) -> PlanningResult:
        backend = backends.resolve(backend_name, timeout_seconds=timeout_seconds)
        return backend.solve(domain_text, problem_text)

    return plan_fn


def goals_from_world(world: WorldModel) -> List[str]:
    """Default mission: clean everything marked dirty in the world file."""
    return sorted(set(world.dirty_locations()) | set(world.deep_dirty_locations()))


def parse_goals_param(raw: Sequence[str], world: WorldModel) -> List[str]:
    """Interpret the ``goals`` parameter: explicit list or ['all']."""
    goals = [g.strip() for g in raw if g.strip()]
    if not goals or goals == ['all']:
        return goals_from_world(world)
    return goals
