import pytest

from washbot_planning.data import read_pddl
from washbot_planning.pddl_core import parse_domain, parse_problem
from washbot_planning.pddl_core.model import Atom, PddlError
from washbot_planning.pddl_core.parser import structural_check, tokenize


def test_tokenize_strips_comments():
    tokens = tokenize('(and ; a comment\n  (foo))')
    assert tokens == ['(', 'and', '(', 'foo', ')', ')']


def test_parse_strips_domain():
    domain = parse_domain(read_pddl('domain_strips.pddl'))
    assert domain.name == 'washbot'
    assert set(domain.actions) == {'move', 'clean'}
    assert set(domain.predicates) == {'robot_at', 'connected', 'dirty', 'cleaned'}
    move = domain.actions['move']
    assert [t for _, t in move.parameters] == ['robot', 'location', 'location']


def test_parse_problem():
    problem = parse_problem(read_pddl('washroom_small.pddl'))
    assert problem.domain_name == 'washbot'
    assert problem.objects['washbot'] == 'robot'
    assert Atom('robot_at', ('washbot', 'dock')) in problem.init
    assert Atom('cleaned', ('basin',)) in problem.goal
    assert len(problem.goal) == 2


def test_type_hierarchy_defaults_to_object():
    domain = parse_domain(read_pddl('domain_strips.pddl'))
    assert domain.type_and_ancestors('robot') == ['robot', 'object']


def test_durative_domain_rejected_with_clear_error():
    # The temporal domain must be refused with a pointer to a capable backend,
    # never silently mis-planned as if it were STRIPS.
    with pytest.raises(PddlError, match='POPF or PlanSys2'):
        parse_domain(read_pddl('domain_temporal.pddl'))


def test_temporal_domain_passes_structural_check():
    structural_check(read_pddl('domain_temporal.pddl'))
    structural_check(read_pddl('washroom_small_temporal.pddl'))


def test_unbalanced_parens_rejected():
    with pytest.raises(PddlError):
        structural_check('(define (domain broken)')


def test_negative_goal_rejected():
    text = """
    (define (problem p) (:domain washbot)
      (:objects r - robot a - location)
      (:init (robot_at r a))
      (:goal (not (robot_at r a))))
    """
    with pytest.raises(PddlError, match='negative'):
        parse_problem(text)
