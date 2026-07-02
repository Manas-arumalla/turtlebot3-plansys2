"""Heuristic forward search over a ground STRIPS task.

Two strategies are provided:

- ``gbfs``  — greedy best-first search on the additive heuristic. Fast and
  the default for large instances; plans may be slightly longer than optimal.
- ``astar`` — A* with the additive heuristic (inadmissible, so plans are
  good but not guaranteed optimal; in practice near-optimal on this domain).

The additive heuristic (h_add) is recomputed per state with a Dijkstra-style
sweep over the relaxed task, which is more than fast enough at the scale of
the cleaning missions (hundreds of locations).
"""

from __future__ import annotations

import heapq
import itertools
import time
from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Optional

from washbot_planning.pddl_core.model import Atom, GroundAction, GroundTask


@dataclass
class SearchResult:
    solved: bool
    plan: List[GroundAction] = field(default_factory=list)
    expanded: int = 0
    generated: int = 0
    time_seconds: float = 0.0
    failure_reason: str = ''

    @property
    def plan_length(self) -> int:
        return len(self.plan)


class _HAdd:
    """Additive heuristic over the delete-relaxed task."""

    def __init__(self, task: GroundTask):
        self._task = task
        # Precompute, for each atom, which actions have it as a precondition.
        self._consumers: Dict[Atom, List[int]] = {}
        self._precondition_counts: List[int] = []
        for index, action in enumerate(task.actions):
            self._precondition_counts.append(len(action.preconditions))
            for atom in action.preconditions:
                self._consumers.setdefault(atom, []).append(index)

    def __call__(self, state: FrozenSet[Atom]) -> float:
        costs: Dict[Atom, float] = {atom: 0.0 for atom in state}
        remaining = list(self._precondition_counts)
        queue: List[tuple] = [(0.0, atom) for atom in state]
        heapq.heapify(queue)
        actions = self._task.actions

        def action_cost(index: int) -> float:
            total = actions[index].cost
            for pre in actions[index].preconditions:
                total += costs[pre]
            return total

        # Actions with no (dynamic) preconditions are immediately applicable.
        for index, count in enumerate(remaining):
            if count == 0:
                for added in actions[index].add_effects:
                    if added not in costs or actions[index].cost < costs[added]:
                        costs[added] = actions[index].cost
                        heapq.heappush(queue, (costs[added], added))

        while queue:
            cost, atom = heapq.heappop(queue)
            if cost > costs.get(atom, float('inf')):
                continue
            for index in self._consumers.get(atom, ()):
                remaining[index] -= 1
                if remaining[index] == 0:
                    new_cost = action_cost(index)
                    for added in actions[index].add_effects:
                        if added not in costs or new_cost < costs[added]:
                            costs[added] = new_cost
                            heapq.heappush(queue, (costs[added], added))

        total = 0.0
        for goal_atom in self._task.goal:
            if goal_atom not in costs:
                return float('inf')
            total += costs[goal_atom]
        return total


def solve(task: GroundTask, strategy: str = 'gbfs',
          max_expansions: int = 500_000,
          timeout_seconds: float = 60.0) -> SearchResult:
    """Search for a plan; returns a :class:`SearchResult` either way."""
    if strategy not in ('gbfs', 'astar'):
        raise ValueError(f'unknown search strategy "{strategy}"')

    start_time = time.perf_counter()
    heuristic = _HAdd(task)
    counter = itertools.count()  # FIFO tie-breaking keeps runs deterministic

    initial_h = heuristic(task.init)
    if initial_h == float('inf'):
        return SearchResult(solved=False, failure_reason='goal unreachable (relaxed)',
                            time_seconds=time.perf_counter() - start_time)

    # Entries: (priority, tie, g, state)
    open_list: List[tuple] = [(initial_h, next(counter), 0.0, task.init)]
    parents: Dict[FrozenSet[Atom], Optional[tuple]] = {task.init: None}
    best_g: Dict[FrozenSet[Atom], float] = {task.init: 0.0}
    expanded = 0
    generated = 1

    while open_list:
        if expanded >= max_expansions:
            return SearchResult(solved=False, expanded=expanded, generated=generated,
                                failure_reason='expansion limit reached',
                                time_seconds=time.perf_counter() - start_time)
        if time.perf_counter() - start_time > timeout_seconds:
            return SearchResult(solved=False, expanded=expanded, generated=generated,
                                failure_reason='timeout',
                                time_seconds=time.perf_counter() - start_time)

        _, _, g, state = heapq.heappop(open_list)
        if g > best_g.get(state, float('inf')):
            continue
        if task.goal_satisfied(state):
            return SearchResult(
                solved=True, plan=_extract_plan(parents, state),
                expanded=expanded, generated=generated,
                time_seconds=time.perf_counter() - start_time)
        expanded += 1

        for action in task.actions:
            if not action.applicable(state):
                continue
            successor = action.apply(state)
            new_g = g + action.cost
            if new_g >= best_g.get(successor, float('inf')):
                continue
            h = heuristic(successor)
            if h == float('inf'):
                continue
            best_g[successor] = new_g
            parents[successor] = (state, action)
            priority = h if strategy == 'gbfs' else new_g + h
            heapq.heappush(open_list, (priority, next(counter), new_g, successor))
            generated += 1

    return SearchResult(solved=False, expanded=expanded, generated=generated,
                        failure_reason='search space exhausted',
                        time_seconds=time.perf_counter() - start_time)


def _extract_plan(parents: Dict, state: FrozenSet[Atom]) -> List[GroundAction]:
    plan: List[GroundAction] = []
    entry = parents[state]
    while entry is not None:
        previous_state, action = entry
        plan.append(action)
        entry = parents[previous_state]
    plan.reverse()
    return plan
