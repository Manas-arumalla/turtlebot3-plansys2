"""Pluggable planner backends behind one interface.

Three ways to obtain a plan, all returning the same :class:`PlanningResult`:

- ``internal-gbfs`` / ``internal-astar`` — the bundled heuristic search.
  Zero external dependencies; STRIPS domains only.
- ``popf`` — the POPF temporal planner as a subprocess (the same planner
  PlanSys2 ships with). Required for the temporal domain.
- The PlanSys2 planner *service* is intentionally not a backend here: it
  needs a spinning ROS node, so it lives in ``planner_client`` /
  ``mission_controller`` where an executor is available.

``resolve('auto')`` picks POPF when it is installed and falls back to the
internal search, so every tool in this package works on a bare machine.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from washbot_planning.pddl_core import (
    ground,
    parse_domain,
    parse_problem,
    solve,
)
from washbot_planning.pddl_core.model import PddlError
from washbot_planning.plan import Plan


@dataclass
class PlanningResult:
    success: bool
    backend: str
    plan: Optional[Plan] = None
    error: str = ''
    stats: Dict[str, float] = field(default_factory=dict)


class InternalBackend:
    """Bundled STRIPS planner (see ``pddl_core.search``)."""

    def __init__(self, strategy: str = 'gbfs',
                 timeout_seconds: float = 60.0,
                 max_expansions: int = 500_000):
        self.strategy = strategy
        self.timeout_seconds = timeout_seconds
        self.max_expansions = max_expansions

    @property
    def name(self) -> str:
        return f'internal-{self.strategy}'

    def solve(self, domain_text: str, problem_text: str) -> PlanningResult:
        try:
            domain = parse_domain(domain_text)
            problem = parse_problem(problem_text)
            task = ground(domain, problem)
        except PddlError as error:
            return PlanningResult(success=False, backend=self.name, error=str(error))

        result = solve(task, strategy=self.strategy,
                       timeout_seconds=self.timeout_seconds,
                       max_expansions=self.max_expansions)
        stats = {
            'time_seconds': result.time_seconds,
            'expanded': float(result.expanded),
            'generated': float(result.generated),
            'ground_actions': float(len(task.actions)),
        }
        if not result.solved:
            return PlanningResult(success=False, backend=self.name,
                                  error=result.failure_reason, stats=stats)
        return PlanningResult(success=True, backend=self.name,
                              plan=Plan.from_ground_actions(result.plan, source=self.name),
                              stats=stats)


class PopfBackend:
    """POPF invoked as a subprocess; handles STRIPS and temporal domains."""

    def __init__(self, binary: Optional[str] = None, timeout_seconds: float = 60.0):
        self.binary = binary or find_popf()
        self.timeout_seconds = timeout_seconds

    @property
    def name(self) -> str:
        return 'popf'

    def solve(self, domain_text: str, problem_text: str) -> PlanningResult:
        if not self.binary:
            return PlanningResult(success=False, backend=self.name,
                                  error='popf binary not found on this machine')
        with tempfile.TemporaryDirectory(prefix='washbot_popf_') as workdir:
            domain_path = os.path.join(workdir, 'domain.pddl')
            problem_path = os.path.join(workdir, 'problem.pddl')
            with open(domain_path, 'w', encoding='utf-8') as handle:
                handle.write(domain_text)
            with open(problem_path, 'w', encoding='utf-8') as handle:
                handle.write(problem_text)
            try:
                completed = subprocess.run(
                    [self.binary, domain_path, problem_path],
                    capture_output=True, text=True, timeout=self.timeout_seconds)
            except subprocess.TimeoutExpired:
                return PlanningResult(success=False, backend=self.name,
                                      error=f'popf timed out after {self.timeout_seconds}s')

        plan = Plan.from_popf_output(completed.stdout, source=self.name)
        if not plan.steps:
            detail = completed.stderr.strip() or completed.stdout.strip()
            return PlanningResult(success=False, backend=self.name,
                                  error=f'popf produced no plan: {detail[-400:]}')
        return PlanningResult(success=True, backend=self.name, plan=plan,
                              stats={'makespan': plan.makespan})


def find_popf() -> Optional[str]:
    """Locate a POPF binary on PATH or inside a sourced/known ROS install."""
    on_path = shutil.which('popf')
    if on_path:
        return on_path
    candidates = []
    ros_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
    for prefix in ros_prefix.split(os.pathsep):
        if prefix:
            candidates.append(os.path.join(prefix, 'lib', 'popf', 'popf'))
    if os.path.isdir('/opt/ros'):
        for distro_dir in sorted(os.listdir('/opt/ros')):
            candidates.append(os.path.join('/opt/ros', distro_dir, 'lib', 'popf', 'popf'))
    for candidate in candidates:
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    return None


def available_backends() -> List[str]:
    names = ['internal-gbfs', 'internal-astar']
    if find_popf():
        names.append('popf')
    return names


def resolve(name: str = 'auto', timeout_seconds: float = 60.0):
    """Instantiate a backend by name.

    ``auto`` prefers POPF when installed; otherwise it falls back to the
    internal A* search, which produces noticeably shorter tours than GBFS at
    mission scale for a few extra milliseconds (see docs/benchmarks.md).
    GBFS remains available by name for very large worlds.
    """
    if name == 'auto':
        name = 'popf' if find_popf() else 'internal-astar'
    if name == 'popf':
        return PopfBackend(timeout_seconds=timeout_seconds)
    if name in ('internal-gbfs', 'internal-astar', 'internal'):
        strategy = 'gbfs' if name in ('internal', 'internal-gbfs') else 'astar'
        return InternalBackend(strategy=strategy, timeout_seconds=timeout_seconds)
    raise ValueError(f'unknown planner backend "{name}" '
                     f'(available: {", ".join(available_backends())}, auto)')
