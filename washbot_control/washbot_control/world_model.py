"""The executor's ground truth: waypoints, connectivity, and mission state.

The world model is loaded from ``config/world.yaml`` and mutated as the
mission progresses (moves complete, fixtures get cleaned, passages get
blocked after navigation failures, battery drains). At any moment it can
emit a :class:`WorldSnapshot` for the problem builder, which is how a fresh
PDDL problem — reflecting exactly what the robot knows *now* — is produced
for every plan and replan.

Pure Python on purpose: no ROS imports, fully unit-testable.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set, Tuple

import yaml

from washbot_planning.problem_builder import WorldSnapshot


@dataclass
class Location:
    name: str
    x: float
    y: float
    yaw: float = 0.0
    dirty: bool = False
    deep_dirty: bool = False
    clean_duration: float = 15.0


@dataclass
class WorldModel:
    frame_id: str
    robot_name: str
    locations: Dict[str, Location]
    edges: Set[Tuple[str, str]]
    robot_at: str
    charger: Optional[str] = None
    nominal_speed: float = 0.2  # m/s, used for travel-time estimates
    battery_pct: float = 100.0
    cleaned: Set[str] = field(default_factory=set)
    blocked: Set[Tuple[str, str]] = field(default_factory=set)

    # ------------------------------------------------------------------ load

    @classmethod
    def from_yaml(cls, path: str) -> 'WorldModel':
        with open(path, 'r', encoding='utf-8') as handle:
            config = yaml.safe_load(handle)
        if 'world' not in config:
            raise ValueError(f'{path} has no top-level "world" section')
        return cls.from_config(config['world'])

    @classmethod
    def from_config(cls, world: dict) -> 'WorldModel':
        locations: Dict[str, Location] = {}
        for name, entry in world['locations'].items():
            locations[name] = Location(
                name=name,
                x=float(entry['x']),
                y=float(entry['y']),
                yaw=float(entry.get('yaw', 0.0)),
                dirty=bool(entry.get('dirty', False)),
                deep_dirty=bool(entry.get('deep_dirty', False)),
                clean_duration=float(entry.get('clean_duration', 15.0)),
            )

        edges: Set[Tuple[str, str]] = set()
        for a, b in world.get('edges', []):
            if a not in locations or b not in locations:
                raise ValueError(f'edge ({a}, {b}) references an unknown location')
            edges.add((a, b))
            edges.add((b, a))

        start = world.get('start') or next(iter(locations))
        if start not in locations:
            raise ValueError(f'start location "{start}" is unknown')
        charger = world.get('charger')
        if charger is not None and charger not in locations:
            raise ValueError(f'charger location "{charger}" is unknown')

        return cls(
            frame_id=world.get('frame_id', 'map'),
            robot_name=world.get('robot_name', 'washbot'),
            locations=locations,
            edges=edges,
            robot_at=start,
            charger=charger,
            nominal_speed=float(world.get('nominal_speed', 0.2)),
            battery_pct=float(world.get('battery_pct', 100.0)),
        )

    # ----------------------------------------------------------------- query

    def pose_of(self, name: str) -> Tuple[float, float, float]:
        location = self.locations[name]
        return location.x, location.y, location.yaw

    def distance(self, a: str, b: str) -> float:
        pa, pb = self.locations[a], self.locations[b]
        return math.hypot(pb.x - pa.x, pb.y - pa.y)

    def travel_time(self, a: str, b: str) -> float:
        return max(1.0, self.distance(a, b) / max(self.nominal_speed, 0.01))

    def active_edges(self) -> List[Tuple[str, str]]:
        return sorted(self.edges - self.blocked)

    def dirty_locations(self) -> List[str]:
        return sorted(
            name for name, loc in self.locations.items()
            if loc.dirty and name not in self.cleaned)

    def deep_dirty_locations(self) -> List[str]:
        return sorted(
            name for name, loc in self.locations.items()
            if loc.deep_dirty and name not in self.cleaned)

    def goals_satisfied(self, goals: Sequence[str]) -> bool:
        return all(goal in self.cleaned for goal in goals)

    # ---------------------------------------------------------------- mutate

    def apply_move(self, destination: str) -> None:
        if destination not in self.locations:
            raise ValueError(f'unknown destination "{destination}"')
        self.robot_at = destination

    def apply_clean(self, location: str) -> None:
        if location not in self.locations:
            raise ValueError(f'unknown location "{location}"')
        self.cleaned.add(location)

    def block_edge(self, a: str, b: str, both_directions: bool = True) -> None:
        self.blocked.add((a, b))
        if both_directions:
            self.blocked.add((b, a))

    def drain_battery(self, amount: float) -> None:
        self.battery_pct = max(0.0, self.battery_pct - amount)

    def recharge(self) -> None:
        self.battery_pct = 100.0

    # -------------------------------------------------------------- snapshot

    def snapshot(self) -> WorldSnapshot:
        edges = self.active_edges()
        travel_times = {(a, b): round(self.travel_time(a, b), 1) for a, b in edges}
        clean_times = {
            name: loc.clean_duration for name, loc in self.locations.items()
        }
        return WorldSnapshot(
            robot=self.robot_name,
            robot_at=self.robot_at,
            locations=sorted(self.locations),
            edges=edges,
            dirty=self.dirty_locations(),
            deep_dirty=self.deep_dirty_locations(),
            cleaned=sorted(self.cleaned),
            charger_at=self.charger,
            battery_pct=self.battery_pct,
            travel_times=travel_times,
            clean_times=clean_times,
        )
