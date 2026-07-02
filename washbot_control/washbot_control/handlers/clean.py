"""Handler for the PDDL ``clean`` / ``deep_clean`` actions.

Cleaning is simulated as a timed routine: the robot holds position (or
slowly spins, which reads nicely in RViz and Gazebo), progress is published
for monitoring, and the battery drains. Swapping this for a real tool —
a vacuum motor, a UV lamp, an arm routine — only requires replacing this
handler; the planner and mission controller are untouched.
"""

from __future__ import annotations

import json
from typing import Optional

from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String

from washbot_control.handlers.base import ActionHandler, DoneCallback
from washbot_planning.plan import PlanStep

DEEP_CLEAN_FACTOR = 2.5


class CleanHandler(ActionHandler):

    def __init__(self, node, world, deep: bool = False,
                 tick_seconds: float = 0.5,
                 battery_drain: float = 5.0,
                 spin_while_cleaning: bool = True,
                 spin_speed: float = 0.6,
                 stamped_cmd_vel: bool = False):
        super().__init__(node, world)
        self._deep = deep
        self._tick_seconds = tick_seconds
        self._battery_drain = battery_drain
        self._spin = spin_while_cleaning
        self._spin_speed = spin_speed
        # The Jazzy TurtleBot3 gz bridge takes TwistStamped on cmd_vel;
        # older stacks (and Nav2 defaults) use plain Twist.
        self._stamped = stamped_cmd_vel
        self._progress_pub = node.create_publisher(
            String, '/washbot/cleaning_progress', 10)
        self._cmd_vel_pub = node.create_publisher(
            TwistStamped if stamped_cmd_vel else Twist, 'cmd_vel', 10)
        self._timer = None
        self._done_cb: Optional[DoneCallback] = None

    def start(self, step: PlanStep, done_cb: DoneCallback) -> None:
        location_name = step.args[-1]
        location = self.world.locations[location_name]
        duration = location.clean_duration * (DEEP_CLEAN_FACTOR if self._deep else 1.0)

        self._done_cb = done_cb
        self._location = location_name
        self._duration = max(duration, self._tick_seconds)
        self._elapsed = 0.0
        label = 'deep-cleaning' if self._deep else 'cleaning'
        self.node.get_logger().info(
            f'{label} "{location_name}" for {self._duration:.0f}s')
        self._timer = self.node.create_timer(self._tick_seconds, self._tick)

    def cancel(self) -> None:
        self._stop_motion()
        self._drop_timer()
        self._done_cb = None

    # ------------------------------------------------------------- internals

    def _tick(self) -> None:
        self._elapsed += self._tick_seconds
        fraction = min(1.0, self._elapsed / self._duration)

        self._progress_pub.publish(String(data=json.dumps({
            'location': self._location,
            'progress': round(fraction, 3),
            'deep': self._deep,
        })))
        if self._spin:
            self._publish_spin(self._spin_speed)

        if fraction >= 1.0:
            self._stop_motion()
            self._drop_timer()
            self.world.drain_battery(self._battery_drain)
            callback, self._done_cb = self._done_cb, None
            if callback is not None:
                callback(True, '')

    def _publish_spin(self, angular_z: float) -> None:
        if self._stamped:
            message = TwistStamped()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.twist.angular.z = angular_z
        else:
            message = Twist()
            message.angular.z = angular_z
        self._cmd_vel_pub.publish(message)

    def _stop_motion(self) -> None:
        if self._spin:
            self._publish_spin(0.0)

    def _drop_timer(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self.node.destroy_timer(self._timer)
            self._timer = None
