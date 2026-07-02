"""Parameterized world generator for benchmarking and stress tests.

Generates facility layouts of arbitrary size: a corridor spine with rooms
branching off it and cleanable fixtures inside each room. The same
:class:`WorldSnapshot` structure the executor uses at runtime is used here,
so benchmark problems exercise exactly the same code path as real missions.
"""

from __future__ import annotations

import random
from typing import List, Tuple

from washbot_planning.problem_builder import WorldSnapshot


def generate_world(rooms: int, fixtures_per_room: int = 2,
                   rooms_per_corridor: int = 2, seed: int = 7) -> WorldSnapshot:
    """Build a synthetic facility world.

    Layout: ``dock — corridor_1 — corridor_2 — ...`` with ``rooms_per_corridor``
    rooms hanging off each corridor segment and ``fixtures_per_room`` cleanable
    fixtures per room. All fixtures start dirty.
    """
    if rooms < 1:
        raise ValueError('rooms must be >= 1')
    if fixtures_per_room < 1:
        raise ValueError('fixtures_per_room must be >= 1')

    rng = random.Random(seed)
    locations: List[str] = ['dock']
    edges: List[Tuple[str, str]] = []
    dirty: List[str] = []

    def connect(a: str, b: str) -> None:
        edges.append((a, b))
        edges.append((b, a))

    corridor_count = max(1, (rooms + rooms_per_corridor - 1) // rooms_per_corridor)
    previous = 'dock'
    corridors: List[str] = []
    for i in range(1, corridor_count + 1):
        corridor = f'corridor_{i}'
        corridors.append(corridor)
        locations.append(corridor)
        connect(previous, corridor)
        previous = corridor

    travel_times = {}
    for room_index in range(1, rooms + 1):
        room = f'room_{room_index}'
        locations.append(room)
        corridor = corridors[(room_index - 1) // rooms_per_corridor]
        connect(room, corridor)
        for fixture_index in range(1, fixtures_per_room + 1):
            fixture = f'fixture_{room_index}_{fixture_index}'
            locations.append(fixture)
            connect(fixture, room)
            dirty.append(fixture)

    for edge in edges:
        travel_times[edge] = round(rng.uniform(5.0, 20.0), 1)

    clean_times = {f: round(rng.uniform(10.0, 30.0), 1) for f in dirty}

    return WorldSnapshot(
        robot='washbot',
        robot_at='dock',
        locations=locations,
        edges=edges,
        dirty=dirty,
        charger_at='dock',
        battery_pct=100.0,
        travel_times=travel_times,
        clean_times=clean_times,
    )


def goal_all_fixtures(snapshot: WorldSnapshot) -> List[str]:
    """The canonical benchmark goal: every dirty fixture cleaned."""
    return list(snapshot.dirty)
