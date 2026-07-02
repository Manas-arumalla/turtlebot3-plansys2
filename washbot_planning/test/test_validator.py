import pytest

from washbot_planning.data import read_pddl
from washbot_planning.pddl_core import parse_domain, parse_problem, validate_plan


@pytest.fixture(scope='module')
def domain():
    return parse_domain(read_pddl('domain_strips.pddl'))


@pytest.fixture(scope='module')
def problem():
    return parse_problem(read_pddl('washroom_small.pddl'))


GOOD_PLAN = [
    ('move', ('washbot', 'dock', 'hall')),
    ('move', ('washbot', 'hall', 'basin')),
    ('clean', ('washbot', 'basin')),
    ('move', ('washbot', 'basin', 'shower')),
    ('move', ('washbot', 'shower', 'commode')),
    ('clean', ('washbot', 'commode')),
]


def test_valid_plan_accepted(domain, problem):
    report = validate_plan(domain, problem, GOOD_PLAN)
    assert report.valid
    assert 'valid' in report.summary()


def test_missing_precondition_detected(domain, problem):
    # Skip the first move: robot is not at 'hall' for the second one.
    report = validate_plan(domain, problem, GOOD_PLAN[1:])
    assert not report.valid
    failed = [s for s in report.steps if not s.ok][0]
    assert '(robot_at washbot hall)' in failed.missing_preconditions


def test_unmet_goal_detected(domain, problem):
    report = validate_plan(domain, problem, GOOD_PLAN[:3])  # basin only
    assert not report.valid
    assert report.unmet_goals == ['(cleaned commode)']


def test_bogus_grounding_detected(domain, problem):
    report = validate_plan(domain, problem, [('move', ('washbot', 'dock', 'basin'))])
    assert not report.valid
    assert 'not a valid grounding' in report.error


def test_double_clean_rejected(domain, problem):
    plan = GOOD_PLAN[:3] + [('clean', ('washbot', 'basin'))] + GOOD_PLAN[3:]
    report = validate_plan(domain, problem, plan)
    assert not report.valid  # (dirty basin) no longer holds
