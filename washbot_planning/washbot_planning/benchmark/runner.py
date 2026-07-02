"""Run planner backends over generated facility problems and record metrics.

Each row of the resulting CSV is one (backend, size, repeat) run with wall
time, plan length, node counts (internal backends), and a validation flag —
every returned plan is replayed through the validator, so a "solved" row
really means a correct plan, not just planner output.
"""

from __future__ import annotations

import csv
import os
import statistics
import time
from typing import Dict, List

from washbot_planning import backends
from washbot_planning.data import read_pddl
from washbot_planning.generator import generate_world, goal_all_fixtures
from washbot_planning.pddl_core import parse_domain, parse_problem, validate_plan
from washbot_planning.problem_builder import build_strips_problem

CSV_FIELDS = [
    'backend', 'rooms', 'fixtures_per_room', 'locations', 'goals',
    'repeat', 'solved', 'valid', 'plan_length', 'wall_time_s',
    'expanded', 'generated', 'ground_actions', 'error',
]


def run_benchmark(sizes: List[int], backend_names: List[str],
                  fixtures_per_room: int = 2, repeats: int = 3,
                  timeout_seconds: float = 60.0,
                  seed: int = 7) -> List[Dict]:
    domain_text = read_pddl('domain_strips.pddl')
    domain = parse_domain(domain_text)
    rows: List[Dict] = []

    for rooms in sizes:
        snapshot = generate_world(rooms=rooms, fixtures_per_room=fixtures_per_room,
                                  seed=seed)
        goals = goal_all_fixtures(snapshot)
        problem_text = build_strips_problem(snapshot, goals,
                                            problem_name=f'bench_{rooms}_rooms')
        problem = parse_problem(problem_text)

        for backend_name in backend_names:
            backend = backends.resolve(backend_name, timeout_seconds=timeout_seconds)
            for repeat in range(repeats):
                started = time.perf_counter()
                result = backend.solve(domain_text, problem_text)
                wall_time = time.perf_counter() - started

                valid = False
                plan_length = 0
                if result.success and result.plan is not None:
                    plan_length = len(result.plan)
                    report = validate_plan(domain, problem, result.plan.as_tuples())
                    valid = report.valid

                rows.append({
                    'backend': result.backend,
                    'rooms': rooms,
                    'fixtures_per_room': fixtures_per_room,
                    'locations': len(snapshot.locations),
                    'goals': len(goals),
                    'repeat': repeat,
                    'solved': int(result.success),
                    'valid': int(valid),
                    'plan_length': plan_length,
                    'wall_time_s': round(wall_time, 6),
                    'expanded': int(result.stats.get('expanded', 0)),
                    'generated': int(result.stats.get('generated', 0)),
                    'ground_actions': int(result.stats.get('ground_actions', 0)),
                    'error': result.error,
                })
    return rows


def write_csv(rows: List[Dict], path: str) -> None:
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def summarize(rows: List[Dict]) -> List[Dict]:
    """Aggregate repeats into per-(backend, rooms) medians for reporting."""
    groups: Dict[tuple, List[Dict]] = {}
    for row in rows:
        groups.setdefault((row['backend'], row['rooms']), []).append(row)

    summary = []
    for (backend, rooms), group in sorted(groups.items()):
        solved = [r for r in group if r['solved']]
        summary.append({
            'backend': backend,
            'rooms': rooms,
            'locations': group[0]['locations'],
            'goals': group[0]['goals'],
            'solve_rate': len(solved) / len(group),
            'median_time_s': statistics.median(r['wall_time_s'] for r in group),
            'median_plan_length':
                statistics.median(r['plan_length'] for r in solved) if solved else 0,
            'median_expanded':
                statistics.median(r['expanded'] for r in solved) if solved else 0,
            'all_valid': all(r['valid'] for r in solved) if solved else False,
        })
    return summary
