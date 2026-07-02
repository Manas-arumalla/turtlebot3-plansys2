import pytest

from washbot_planning.data import read_pddl
from washbot_planning.pddl_core import ground, parse_domain, parse_problem, solve


@pytest.fixture(scope='module')
def domain():
    return parse_domain(read_pddl('domain_strips.pddl'))


@pytest.fixture(scope='module')
def problem():
    return parse_problem(read_pddl('washroom_small.pddl'))


def test_static_pruning_limits_move_actions(domain, problem):
    task = ground(domain, problem)
    moves = [a for a in task.actions if a.name == 'move']
    # One ground move per declared connection (10 directed edges),
    # not one per location pair (5 * 5 = 25).
    assert len(moves) == 10
    # 'connected' is static: it must not appear in any dynamic precondition.
    for action in task.actions:
        assert all(atom.predicate != 'connected' for atom in action.preconditions)


def test_astar_finds_optimal_plan(domain, problem):
    task = ground(domain, problem)
    result = solve(task, strategy='astar')
    assert result.solved
    # dock->hall->basin (clean) ->hall/shower route-> commode (clean) = 6 steps.
    assert result.plan_length == 6
    assert [a.name for a in result.plan].count('clean') == 2


def test_gbfs_solves(domain, problem):
    result = solve(ground(domain, problem), strategy='gbfs')
    assert result.solved
    assert result.plan_length >= 6


def test_unreachable_goal_reported(domain):
    text = """
    (define (problem stranded) (:domain washbot)
      (:objects r - robot a b - location)
      (:init (robot_at r a) (dirty b))
      (:goal (and (cleaned b))))
    """
    result = solve(ground(domain, parse_problem(text)))
    assert not result.solved
    assert 'unreachable' in result.failure_reason


def test_expansion_limit_respected(domain, problem):
    result = solve(ground(domain, problem), max_expansions=0)
    assert not result.solved
    assert 'limit' in result.failure_reason
