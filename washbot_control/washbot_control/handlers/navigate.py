"""Handler for the PDDL ``move`` action: drives Nav2's NavigateToPose.

Works identically against the real Nav2 stack and against the simulated
server in ``washbot_control.sim.fake_nav2_server`` — both expose the same
action interface, which is the whole point of the fake.
"""

from __future__ import annotations

import math
from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from washbot_control.handlers.base import ActionHandler, DoneCallback
from washbot_planning.plan import PlanStep


def yaw_to_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class NavigateHandler(ActionHandler):

    def __init__(self, node, world, action_name: str = 'navigate_to_pose',
                 server_wait_seconds: float = 10.0,
                 battery_drain_per_meter: float = 1.5):
        super().__init__(node, world)
        self._client = ActionClient(node, NavigateToPose, action_name)
        self._server_wait_seconds = server_wait_seconds
        self._battery_drain_per_meter = battery_drain_per_meter
        self._goal_handle = None
        self._done_cb: Optional[DoneCallback] = None
        self._wait_timer = None
        self._waited = 0.0
        self._distance_remaining = 0.0

    # ------------------------------------------------------------------ api

    def start(self, step: PlanStep, done_cb: DoneCallback) -> None:
        self._done_cb = done_cb
        self._step = step
        destination = step.args[-1]
        origin = step.args[-2]
        self._travel_distance = self.world.distance(origin, destination)

        if self._client.server_is_ready():
            self._send_goal(destination)
            return
        # Poll for the action server without blocking the executor.
        self._waited = 0.0
        self._destination = destination
        self._wait_timer = self.node.create_timer(0.5, self._check_server)

    def cancel(self) -> None:
        # Drop the callback first so a late result cannot be mis-attributed
        # to whichever step is current by the time it arrives.
        self._done_cb = None
        self._drop_wait_timer()
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001 - cancellation is best-effort
                pass

    # ------------------------------------------------------------- internals

    def _check_server(self) -> None:
        self._waited += 0.5
        if self._client.server_is_ready():
            self._drop_wait_timer()
            self._send_goal(self._destination)
        elif self._waited >= self._server_wait_seconds:
            self._drop_wait_timer()
            self._finish(False, 'NavigateToPose action server not available')

    def _drop_wait_timer(self) -> None:
        if self._wait_timer is not None:
            self._wait_timer.cancel()
            self.node.destroy_timer(self._wait_timer)
            self._wait_timer = None

    def _send_goal(self, destination: str) -> None:
        x, y, yaw = self.world.pose_of(destination)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.world.frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.node.get_logger().info(
            f'navigating to "{destination}" ({x:.2f}, {y:.2f}, yaw {yaw:.2f})')
        future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, feedback_msg) -> None:
        self._distance_remaining = feedback_msg.feedback.distance_remaining

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._finish(False, 'navigation goal rejected by the server')
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        self._goal_handle = None
        try:
            wrapped = future.result()
        except Exception as error:  # noqa: BLE001 - surface any client failure
            self._finish(False, f'navigation result error: {error}')
            return
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.world.drain_battery(self._travel_distance
                                     * self._battery_drain_per_meter)
            self._finish(True, '')
        else:
            status_names = {
                GoalStatus.STATUS_ABORTED: 'aborted',
                GoalStatus.STATUS_CANCELED: 'canceled',
            }
            status = status_names.get(wrapped.status, f'status {wrapped.status}')
            self._finish(False, f'navigation {status} '
                                f'({self._distance_remaining:.2f} m remaining)')

    def _finish(self, success: bool, message: str) -> None:
        callback, self._done_cb = self._done_cb, None
        if callback is not None:
            callback(success, message)
