import math
import os

import pytest

from washbot_control.world_model import WorldModel

CONFIG_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           'config', 'world.yaml')


@pytest.fixture
def world():
    return WorldModel.from_yaml(CONFIG_PATH)


def test_loads_shipped_world_file(world):
    assert world.robot_at == 'dock'
    assert world.charger == 'dock'
    assert set(world.locations) == {'dock', 'hall', 'basin', 'commode', 'shower'}
    # 5 declared passages -> 10 directed edges.
    assert len(world.edges) == 10


def test_dirty_lists_split_by_depth(world):
    assert world.dirty_locations() == ['basin', 'commode']
    assert world.deep_dirty_locations() == ['shower']


def test_pose_and_distance(world):
    x, y, yaw = world.pose_of('dock')
    assert (x, y) == (-2.05, -0.55)
    assert math.isclose(world.distance('dock', 'hall'), 1.5)
    assert world.travel_time('dock', 'hall') == pytest.approx(6.0)


def test_mutations_flow_into_snapshot(world):
    world.apply_move('hall')
    world.apply_clean('basin')
    world.block_edge('hall', 'commode')
    world.drain_battery(30.0)

    snapshot = world.snapshot()
    assert snapshot.robot_at == 'hall'
    assert 'basin' in snapshot.cleaned
    assert 'basin' not in snapshot.dirty
    assert ('hall', 'commode') not in snapshot.edges
    assert ('commode', 'hall') not in snapshot.edges  # blocked both ways
    assert snapshot.battery_pct == 70.0


def test_goals_satisfied(world):
    assert not world.goals_satisfied(['basin'])
    world.apply_clean('basin')
    assert world.goals_satisfied(['basin'])


def test_unknown_references_rejected():
    with pytest.raises(ValueError, match='unknown location'):
        WorldModel.from_config({
            'locations': {'a': {'x': 0, 'y': 0}},
            'edges': [['a', 'b']],
        })
