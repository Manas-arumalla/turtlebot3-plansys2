"""Generate PDDL problem files from a structured world snapshot.

The executor never edits PDDL text by hand: after every world-model change
(a blocked passage, a freshly cleaned fixture, a battery update) it rebuilds
the problem from the current :class:`WorldSnapshot` and asks the planner
again. Keeping problem generation in one tested place is what makes
replanning reliable.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

DEFAULT_TRAVEL_TIME = 10.0
DEFAULT_CLEAN_TIME = 15.0
DEFAULT_DEEP_CLEAN_TIME = 40.0
DEFAULT_TRAVEL_DRAIN = 2.0
DEFAULT_CLEAN_DRAIN = 5.0


@dataclass
class WorldSnapshot:
    """Everything the planner needs to know about the world right now."""

    robot: str
    robot_at: str
    locations: List[str]
    edges: List[Tuple[str, str]]  # directed; add both directions for two-way
    dirty: List[str] = field(default_factory=list)
    deep_dirty: List[str] = field(default_factory=list)
    cleaned: List[str] = field(default_factory=list)
    charger_at: Optional[str] = None
    battery_pct: float = 100.0
    travel_times: Dict[Tuple[str, str], float] = field(default_factory=dict)
    clean_times: Dict[str, float] = field(default_factory=dict)

    def validate(self) -> None:
        known = set(self.locations)
        if self.robot_at not in known:
            raise ValueError(f'robot location "{self.robot_at}" is not a known location')
        for a, b in self.edges:
            if a not in known or b not in known:
                raise ValueError(f'edge ({a}, {b}) references an unknown location')
        for group_name, group in (('dirty', self.dirty),
                                  ('deep_dirty', self.deep_dirty),
                                  ('cleaned', self.cleaned)):
            for loc in group:
                if loc not in known:
                    raise ValueError(f'{group_name} location "{loc}" is unknown')
        if self.charger_at is not None and self.charger_at not in known:
            raise ValueError(f'charger location "{self.charger_at}" is unknown')


def _check_goals(snapshot: WorldSnapshot, goals: Sequence[str]) -> None:
    known = set(snapshot.locations)
    for goal in goals:
        if goal not in known:
            raise ValueError(f'goal location "{goal}" is unknown')


def build_strips_problem(snapshot: WorldSnapshot, goals: Sequence[str],
                         problem_name: str = 'washbot_mission') -> str:
    """Emit a problem for the STRIPS domain (``washbot``)."""
    snapshot.validate()
    _check_goals(snapshot, goals)

    lines = [f'(define (problem {problem_name})',
             '  (:domain washbot)',
             '  (:objects']
    lines.append(f'    {snapshot.robot} - robot')
    lines.append('    {} - location'.format(' '.join(snapshot.locations)))
    lines.append('  )')
    lines.append('  (:init')
    lines.append(f'    (robot_at {snapshot.robot} {snapshot.robot_at})')
    for a, b in snapshot.edges:
        lines.append(f'    (connected {a} {b})')
    for loc in snapshot.dirty:
        lines.append(f'    (dirty {loc})')
    for loc in snapshot.cleaned:
        lines.append(f'    (cleaned {loc})')
    lines.append('  )')
    goal_atoms = ' '.join(f'(cleaned {g})' for g in goals)
    lines.append(f'  (:goal (and {goal_atoms}))')
    lines.append(')')
    return '\n'.join(lines) + '\n'


def build_temporal_problem(snapshot: WorldSnapshot, goals: Sequence[str],
                           problem_name: str = 'washbot_mission') -> str:
    """Emit a problem for the temporal domain (``washbot_temporal``)."""
    snapshot.validate()
    _check_goals(snapshot, goals)

    lines = [f'(define (problem {problem_name})',
             '  (:domain washbot_temporal)',
             '  (:objects']
    lines.append(f'    {snapshot.robot} - robot')
    lines.append('    {} - location'.format(' '.join(snapshot.locations)))
    lines.append('  )')
    lines.append('  (:init')
    lines.append(f'    (robot_at {snapshot.robot} {snapshot.robot_at})')
    for a, b in snapshot.edges:
        lines.append(f'    (connected {a} {b})')
    for loc in snapshot.dirty:
        lines.append(f'    (dirty {loc})')
    for loc in snapshot.deep_dirty:
        lines.append(f'    (deep_dirty {loc})')
    for loc in snapshot.cleaned:
        lines.append(f'    (cleaned {loc})')
    if snapshot.charger_at:
        lines.append(f'    (charger_at {snapshot.charger_at})')
    lines.append(f'    (= (battery-level {snapshot.robot}) {snapshot.battery_pct:.1f})')
    for a, b in snapshot.edges:
        travel = snapshot.travel_times.get((a, b), DEFAULT_TRAVEL_TIME)
        lines.append(f'    (= (travel-time {a} {b}) {travel:.1f})')
        lines.append(f'    (= (travel-drain {a} {b}) {DEFAULT_TRAVEL_DRAIN:.1f})')
    for loc in snapshot.dirty:
        clean = snapshot.clean_times.get(loc, DEFAULT_CLEAN_TIME)
        lines.append(f'    (= (clean-time {loc}) {clean:.1f})')
    for loc in snapshot.deep_dirty:
        clean = snapshot.clean_times.get(loc, DEFAULT_DEEP_CLEAN_TIME)
        lines.append(f'    (= (deep-clean-time {loc}) {clean:.1f})')
    # One drain assignment per location, even if it is somehow flagged both
    # dirty and deep_dirty — duplicate fluent assignments are a planner error.
    for loc in sorted(set(snapshot.dirty) | set(snapshot.deep_dirty)):
        lines.append(f'    (= (clean-drain {loc}) {DEFAULT_CLEAN_DRAIN:.1f})')
    lines.append('  )')
    goal_atoms = ' '.join(f'(cleaned {g})' for g in goals)
    lines.append(f'  (:goal (and {goal_atoms}))')
    lines.append('  (:metric minimize (total-time))')
    lines.append(')')
    return '\n'.join(lines) + '\n'
