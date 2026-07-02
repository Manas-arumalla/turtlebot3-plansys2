"""Planner-agnostic plan representation.

Whatever produces the plan — the internal search, a POPF subprocess, or the
PlanSys2 planner service — it is normalized into a :class:`Plan` so the
executor, validator, and reports all speak one format.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Iterable, List, Sequence, Tuple

from washbot_planning.pddl_core.model import GroundAction

_POPF_LINE = re.compile(
    r'^\s*(?P<time>\d+(?:\.\d+)?)\s*:\s*\(\s*(?P<body>[^)]+?)\s*\)'
    r'\s*\[\s*(?P<duration>\d+(?:\.\d+)?)\s*\]\s*$')


@dataclass(frozen=True)
class PlanStep:
    index: int
    name: str
    args: Tuple[str, ...]
    start_time: float = 0.0
    duration: float = 0.0

    @property
    def signature(self) -> str:
        return '({} {})'.format(self.name, ' '.join(self.args)) if self.args \
            else '({})'.format(self.name)


@dataclass
class Plan:
    steps: List[PlanStep] = field(default_factory=list)
    source: str = ''

    def __len__(self) -> int:
        return len(self.steps)

    def __iter__(self):
        return iter(self.steps)

    @property
    def makespan(self) -> float:
        if not self.steps:
            return 0.0
        return max(step.start_time + step.duration for step in self.steps)

    def as_tuples(self) -> List[Tuple[str, Tuple[str, ...]]]:
        """The (name, args) view the validator consumes."""
        return [(step.name, step.args) for step in self.steps]

    def to_text(self) -> str:
        lines = [
            f'{step.start_time:.3f}: {step.signature}  [{step.duration:.3f}]'
            for step in self.steps
        ]
        return '\n'.join(lines)

    @classmethod
    def from_ground_actions(cls, actions: Sequence[GroundAction],
                            source: str = 'internal') -> 'Plan':
        steps = [
            PlanStep(index=i, name=a.name, args=a.args,
                     start_time=float(i), duration=1.0)
            for i, a in enumerate(actions)
        ]
        return cls(steps=steps, source=source)

    @classmethod
    def from_popf_output(cls, text: str, source: str = 'popf') -> 'Plan':
        """Parse POPF/OPTIC-style output lines: ``0.000: (move r1 a b)  [10.0]``."""
        steps: List[PlanStep] = []
        for line in text.splitlines():
            match = _POPF_LINE.match(line)
            if not match:
                continue
            tokens = match.group('body').split()
            steps.append(PlanStep(
                index=len(steps),
                name=tokens[0],
                args=tuple(tokens[1:]),
                start_time=float(match.group('time')),
                duration=float(match.group('duration')),
            ))
        steps.sort(key=lambda s: (s.start_time, s.index))
        steps = [
            PlanStep(index=i, name=s.name, args=s.args,
                     start_time=s.start_time, duration=s.duration)
            for i, s in enumerate(steps)
        ]
        return cls(steps=steps, source=source)

    @classmethod
    def from_plansys2_items(cls, items: Iterable, source: str = 'plansys2') -> 'Plan':
        """Build a plan from ``plansys2_msgs/Plan.items`` (duck-typed)."""
        steps: List[PlanStep] = []
        for item in items:
            tokens = item.action.strip().lstrip('(').rstrip(')').split()
            steps.append(PlanStep(
                index=len(steps),
                name=tokens[0],
                args=tuple(tokens[1:]),
                start_time=float(item.time),
                duration=float(item.duration),
            ))
        return cls(steps=steps, source=source)
