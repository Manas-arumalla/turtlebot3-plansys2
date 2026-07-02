import pytest

from washbot_planning.backends import InternalBackend
from washbot_planning.data import read_pddl
from washbot_planning.pddl_core import parse_problem
from washbot_planning.pddl_core.parser import structural_check
from washbot_planning.problem_builder import (
    WorldSnapshot,
    build_strips_problem,
    build_temporal_problem,
)


@pytest.fixture
def snapshot():
    return WorldSnapshot(
        robot='washbot',
        robot_at='dock',
        locations=['dock', 'hall', 'basin'],
        edges=[('dock', 'hall'), ('hall', 'dock'),
               ('hall', 'basin'), ('basin', 'hall')],
        dirty=['basin'],
        charger_at='dock',
        battery_pct=42.0,
    )


def test_strips_problem_is_parseable_and_solvable(snapshot):
    text = build_strips_problem(snapshot, ['basin'])
    problem = parse_problem(text)
    assert problem.domain_name == 'washbot'
    result = InternalBackend('astar').solve(read_pddl('domain_strips.pddl'), text)
    assert result.success
    assert len(result.plan) == 3  # move, move, clean


def test_cleaned_state_carries_over(snapshot):
    snapshot.cleaned = ['basin']
    snapshot.dirty = []
    text = build_strips_problem(snapshot, ['basin'])
    assert '(cleaned basin)' in text
    # Goal already satisfied in init -> empty-plan case is the engine's job;
    # the planner should still succeed trivially.
    result = InternalBackend('astar').solve(read_pddl('domain_strips.pddl'), text)
    assert result.success


def test_temporal_problem_structure(snapshot):
    text = build_temporal_problem(snapshot, ['basin'])
    structural_check(text)
    assert '(= (battery-level washbot) 42.0)' in text
    assert '(charger_at dock)' in text
    assert '(:metric minimize (total-time))' in text
    assert '(= (travel-time dock hall)' in text


def test_unknown_goal_rejected(snapshot):
    with pytest.raises(ValueError, match='goal location'):
        build_strips_problem(snapshot, ['garage'])


def test_unknown_edge_rejected(snapshot):
    snapshot.edges.append(('hall', 'attic'))
    with pytest.raises(ValueError, match='unknown location'):
        build_strips_problem(snapshot, ['basin'])
