"""Plan validation by forward simulation of STRIPS semantics.

Every plan — whether produced by the internal search, POPF, or PlanSys2 —
can be replayed against the domain to confirm each action's preconditions
hold and the final state satisfies the goal. The executor runs this before
dispatching a plan to the robot, and the test suite uses it to keep the
PDDL files honest.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Sequence, Tuple

from washbot_planning.pddl_core.grounding import ground
from washbot_planning.pddl_core.model import Atom, Domain, GroundTask, Problem


@dataclass
class StepReport:
    index: int
    signature: str
    ok: bool
    missing_preconditions: List[str] = field(default_factory=list)


@dataclass
class ValidationReport:
    valid: bool
    steps: List[StepReport] = field(default_factory=list)
    unmet_goals: List[str] = field(default_factory=list)
    error: str = ''

    def summary(self) -> str:
        if self.valid:
            return f'plan valid ({len(self.steps)} steps)'
        if self.error:
            return f'plan invalid: {self.error}'
        failed = [s for s in self.steps if not s.ok]
        if failed:
            first = failed[0]
            return (f'plan invalid at step {first.index} {first.signature}: '
                    f'missing {", ".join(first.missing_preconditions)}')
        return 'plan invalid: goal not satisfied ' \
               f'(unmet: {", ".join(self.unmet_goals)})'


def _index_actions(task: GroundTask) -> Dict[Tuple[str, Tuple[str, ...]], int]:
    return {(a.name, a.args): i for i, a in enumerate(task.actions)}


def validate_plan(domain: Domain, problem: Problem,
                  plan: Sequence[Tuple[str, Sequence[str]]]) -> ValidationReport:
    """Validate ``plan`` — a sequence of (action_name, args) pairs."""
    task = ground(domain, problem)
    lookup = _index_actions(task)
    state: FrozenSet[Atom] = task.init
    report = ValidationReport(valid=True)

    for index, (name, args) in enumerate(plan, start=1):
        key = (name, tuple(args))
        signature = '({} {})'.format(name, ' '.join(args)) if args else f'({name})'
        if key not in lookup:
            report.valid = False
            report.error = f'step {index} {signature} is not a valid grounding'
            return report
        action = task.actions[lookup[key]]
        missing = sorted(str(a) for a in action.preconditions - state)
        step = StepReport(index=index, signature=signature,
                          ok=not missing, missing_preconditions=missing)
        report.steps.append(step)
        if missing:
            report.valid = False
            return report
        state = action.apply(state)

    unmet = sorted(str(a) for a in task.goal - state)
    if unmet:
        report.valid = False
        report.unmet_goals = unmet
    return report
