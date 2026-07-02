"""RViz visualization of the world model and the active plan.

Publishes the waypoint graph with live mission state: dirty fixtures in
red, cleaned ones in gold, the charger in green, blocked passages as red
dashed segments, and the remaining planned route as a highlighted polyline.
"""

from __future__ import annotations

from typing import List, Optional

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from washbot_control.world_model import WorldModel
from washbot_planning.plan import Plan

_COLORS = {
    'charger': ColorRGBA(r=0.16, g=0.66, b=0.16, a=1.0),
    'dirty': ColorRGBA(r=0.84, g=0.15, b=0.16, a=1.0),
    'cleaned': ColorRGBA(r=0.98, g=0.75, b=0.14, a=1.0),
    'waypoint': ColorRGBA(r=0.12, g=0.47, b=0.71, a=1.0),
    'edge': ColorRGBA(r=0.55, g=0.55, b=0.55, a=0.8),
    'blocked': ColorRGBA(r=0.84, g=0.15, b=0.16, a=0.9),
    'route': ColorRGBA(r=0.12, g=0.47, b=0.71, a=0.9),
    'label': ColorRGBA(r=0.1, g=0.1, b=0.1, a=1.0),
}

MARKER_NS = 'washbot_world'


def _point(x: float, y: float, z: float = 0.05) -> Point:
    return Point(x=float(x), y=float(y), z=float(z))


def build_markers(world: WorldModel, stamp,
                  plan: Optional[Plan] = None,
                  next_step_index: int = 0) -> MarkerArray:
    markers: List[Marker] = []
    marker_id = 0

    def new_marker(marker_type: int) -> Marker:
        nonlocal marker_id
        marker = Marker()
        marker.header.frame_id = world.frame_id
        marker.header.stamp = stamp
        marker.ns = MARKER_NS
        marker.id = marker_id
        marker_id += 1
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    # --- edges (one LINE_LIST for active, one for blocked) ---
    active = new_marker(Marker.LINE_LIST)
    active.scale.x = 0.02
    active.color = _COLORS['edge']
    seen = set()
    for a, b in world.active_edges():
        if (b, a) in seen:
            continue
        seen.add((a, b))
        xa, ya, _ = world.pose_of(a)
        xb, yb, _ = world.pose_of(b)
        active.points.extend([_point(xa, ya), _point(xb, yb)])
    markers.append(active)

    if world.blocked:
        blocked = new_marker(Marker.LINE_LIST)
        blocked.scale.x = 0.03
        blocked.color = _COLORS['blocked']
        seen = set()
        for a, b in sorted(world.blocked):
            if (b, a) in seen:
                continue
            seen.add((a, b))
            xa, ya, _ = world.pose_of(a)
            xb, yb, _ = world.pose_of(b)
            blocked.points.extend([_point(xa, ya, 0.08), _point(xb, yb, 0.08)])
        markers.append(blocked)

    # --- remaining planned route ---
    if plan is not None and next_step_index < len(plan.steps):
        route = new_marker(Marker.LINE_STRIP)
        route.scale.x = 0.045
        route.color = _COLORS['route']
        x, y, _ = world.pose_of(world.robot_at)
        route.points.append(_point(x, y, 0.1))
        for step in plan.steps[next_step_index:]:
            if step.name == 'move':
                x, y, _ = world.pose_of(step.args[-1])
                route.points.append(_point(x, y, 0.1))
        if len(route.points) > 1:
            markers.append(route)

    # --- waypoints and labels ---
    for name in sorted(world.locations):
        location = world.locations[name]
        if name in world.cleaned:
            color = _COLORS['cleaned']
        elif name == world.charger:
            color = _COLORS['charger']
        elif location.dirty or location.deep_dirty:
            color = _COLORS['dirty']
        else:
            color = _COLORS['waypoint']

        sphere = new_marker(Marker.SPHERE)
        sphere.pose.position = _point(location.x, location.y, 0.1)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.22
        sphere.color = color
        markers.append(sphere)

        label = new_marker(Marker.TEXT_VIEW_FACING)
        label.pose.position = _point(location.x, location.y, 0.42)
        label.scale.z = 0.17
        label.color = _COLORS['label']
        label.text = name
        markers.append(label)

    return MarkerArray(markers=markers)


def clear_markers(world: WorldModel, stamp) -> MarkerArray:
    marker = Marker()
    marker.header.frame_id = world.frame_id
    marker.header.stamp = stamp
    marker.ns = MARKER_NS
    marker.action = Marker.DELETEALL
    return MarkerArray(markers=[marker])
