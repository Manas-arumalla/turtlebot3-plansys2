"""Recovery-behaviour tests for the mission engine.

These drive the full plan -> execute -> monitor -> replan loop with the real
planner and real PDDL domain, simulating step outcomes — the exact logic the
robot runs, minus ROS.
"""

from washbot_control.engine import MissionEngine, MissionState, make_local_plan_fn
from washbot_control.world_model import WorldModel
from washbot_planning.data import read_pddl

WORLD = {
    'frame_id': 'map',
    'robot_name': 'washbot',
    'start': 'dock',
    'charger': 'dock',
    'locations': {
        'dock': {'x': -2.05, 'y': -0.55},
        'hall': {'x': -0.55, 'y': -0.55},
        'basin': {'x': -0.55, 'y': 1.8, 'dirty': True},
        'commode': {'x': 1.8, 'y': -0.55, 'dirty': True},
        'shower': {'x': 1.8, 'y': 1.2},
    },
    'edges': [
        ['dock', 'hall'], ['hall', 'basin'], ['hall', 'commode'],
        ['basin', 'shower'], ['commode', 'shower'],
    ],
}


def make_engine(**overrides) -> MissionEngine:
    defaults = dict(
        world=WorldModel.from_config(WORLD),
        goals=['basin', 'commode'],
        domain_text=read_pddl('domain_strips.pddl'),
        plan_fn=make_local_plan_fn('internal-astar'),
        max_replans=3,
        max_step_retries=1,
    )
    defaults.update(overrides)
    return MissionEngine(**defaults)


def run_to_completion(engine: MissionEngine, outcomes=None, max_steps=100):
    """Drive the engine, failing steps whose signature appears in ``outcomes``.

    ``outcomes`` maps a step signature to a list of results consumed one per
    attempt, e.g. {'(move washbot hall commode)': [False]} fails once.
    """
    outcomes = dict(outcomes or {})
    executed = []
    for _ in range(max_steps):
        step = engine.current_step()
        if step is None:
            break
        queue = outcomes.get(step.signature)
        success = queue.pop(0) if queue else True
        executed.append((step.signature, success))
        engine.report_step_result(success, '' if success else 'simulated failure')
    return executed


def test_happy_path_cleans_everything():
    engine = make_engine()
    assert engine.start()
    executed = run_to_completion(engine)
    assert engine.state is MissionState.SUCCEEDED
    assert engine.world.cleaned == {'basin', 'commode'}
    assert engine.replans == 0
    assert len(executed) == 6  # optimal plan for this world


def test_blocked_passage_triggers_replan_via_detour():
    engine = make_engine()
    assert engine.start()
    # Every attempt to drive hall -> commode fails: the passage is blocked.
    run_to_completion(engine, outcomes={
        '(move washbot hall commode)': [False, False, False],
    })
    assert engine.state is MissionState.SUCCEEDED
    assert engine.replans >= 1
    assert ('hall', 'commode') in engine.world.blocked
    # The detour through the shower loop still got the commode cleaned.
    assert 'commode' in engine.world.cleaned


def test_clean_failure_retried_then_succeeds():
    engine = make_engine()
    assert engine.start()
    executed = run_to_completion(engine, outcomes={
        '(clean washbot basin)': [False, True],
    })
    assert engine.state is MissionState.SUCCEEDED
    attempts = [sig for sig, _ in executed if sig == '(clean washbot basin)']
    assert len(attempts) == 2
    assert any(e.kind == 'retry' for e in engine.events)


def test_persistent_clean_failure_aborts():
    engine = make_engine(max_step_retries=1)
    assert engine.start()
    run_to_completion(engine, outcomes={
        '(clean washbot basin)': [False, False, False],
    })
    assert engine.state is MissionState.ABORTED
    assert 'no retries left' in engine.abort_reason


def test_unreachable_goal_aborts_with_planning_error():
    engine = make_engine()
    assert engine.start()
    # Both passages into the commode fail -> no route remains -> abort.
    run_to_completion(engine, outcomes={
        '(move washbot hall commode)': [False] * 5,
        '(move washbot shower commode)': [False] * 5,
    })
    assert engine.state is MissionState.ABORTED
    assert 'planning failed' in engine.abort_reason
    assert 'basin' in engine.world.cleaned or 'commode' in engine.world.cleaned


def test_replan_limit_enforced():
    engine = make_engine(max_replans=1)
    assert engine.start()
    run_to_completion(engine, outcomes={
        '(move washbot hall commode)': [False] * 5,
        '(move washbot shower commode)': [False] * 5,
        '(move washbot basin shower)': [False] * 5,
    })
    assert engine.state is MissionState.ABORTED
    assert 'replan limit' in engine.abort_reason or 'planning failed' in engine.abort_reason


def test_goals_already_satisfied_short_circuits():
    engine = make_engine()
    engine.world.apply_clean('basin')
    engine.world.apply_clean('commode')
    assert not engine.start()
    assert engine.state is MissionState.SUCCEEDED


def test_unknown_goal_rejected():
    engine = make_engine(goals=['penthouse'])
    assert not engine.start()
    assert engine.state is MissionState.ABORTED
    assert 'unknown goal' in engine.abort_reason


def test_summary_shape():
    engine = make_engine()
    engine.start()
    run_to_completion(engine)
    summary = engine.summary()
    assert summary['state'] == 'succeeded'
    assert summary['cleaned'] == ['basin', 'commode']
    assert summary['steps_executed'] == summary['plan_steps']
    assert any(e['kind'] == 'success' for e in summary['events'])


def test_deep_dirty_folds_into_strips_mode():
    world_cfg = dict(WORLD)
    world_cfg['locations'] = dict(WORLD['locations'])
    world_cfg['locations']['shower'] = {
        'x': 1.8, 'y': 1.2, 'deep_dirty': True}
    engine = make_engine(world=WorldModel.from_config(world_cfg),
                         goals=['basin', 'commode', 'shower'])
    assert engine.start()
    run_to_completion(engine)
    assert engine.state is MissionState.SUCCEEDED
    assert 'shower' in engine.world.cleaned
