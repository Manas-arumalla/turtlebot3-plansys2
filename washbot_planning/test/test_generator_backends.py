import pytest

from washbot_planning import backends
from washbot_planning.data import read_pddl
from washbot_planning.generator import generate_world, goal_all_fixtures
from washbot_planning.pddl_core import parse_domain, parse_problem, validate_plan
from washbot_planning.problem_builder import build_strips_problem


def test_generator_is_deterministic():
    a = generate_world(rooms=4, seed=11)
    b = generate_world(rooms=4, seed=11)
    assert a.locations == b.locations
    assert a.travel_times == b.travel_times


def test_generated_world_scales():
    snapshot = generate_world(rooms=6, fixtures_per_room=3)
    assert len(goal_all_fixtures(snapshot)) == 18
    # dock + 3 corridors + 6 rooms + 18 fixtures
    assert len(snapshot.locations) == 28


def test_generated_problem_solves_and_validates():
    snapshot = generate_world(rooms=3)
    goals = goal_all_fixtures(snapshot)
    domain_text = read_pddl('domain_strips.pddl')
    problem_text = build_strips_problem(snapshot, goals)

    result = backends.resolve('internal-astar').solve(domain_text, problem_text)
    assert result.success
    report = validate_plan(parse_domain(domain_text), parse_problem(problem_text),
                           result.plan.as_tuples())
    assert report.valid


def test_resolve_rejects_unknown_backend():
    with pytest.raises(ValueError, match='unknown planner backend'):
        backends.resolve('sat-solver')


def test_auto_backend_always_available():
    backend = backends.resolve('auto')
    # POPF where installed; otherwise the internal A* (best plan quality
    # at mission scale — see docs/benchmarks.md).
    assert backend.name in ('popf', 'internal-astar')


def test_internal_backend_reports_pddl_errors():
    result = backends.resolve('internal-gbfs').solve('(define (domain', '(define')
    assert not result.success
    assert result.error
