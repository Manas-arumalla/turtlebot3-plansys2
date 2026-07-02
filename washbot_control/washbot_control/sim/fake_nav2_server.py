"""Simulated Nav2 ``NavigateToPose`` server with configurable fault injection.

Exposes exactly the action interface Nav2's bt_navigator exposes, so the
mission controller cannot tell the difference. The robot is a point moving
at constant speed with live feedback (current pose, distance remaining).

Fault injection is what makes it useful beyond a happy-path demo:

- ``fail_edges``: directed passages that cannot be traversed, as
  ``['hall->commode']`` — fails only when driving from hall to the commode.
  Models a blocked corridor; the mission should detour around it.
- ``fail_locations``: waypoint names where *every* approach fails. Models an
  unreachable goal; the mission should exhaust routes and abort cleanly.
- ``fail_mode``: ``abort`` (goal accepted, then fails mid-drive — Nav2's
  behaviour when the robot gets stuck) or ``reject`` (goal refused outright).
- ``fail_times``: how many attempts fail before succeeding; ``-1`` = always.

Example — block the hall-to-commode corridor:

    ros2 run washbot_control fake_nav2_server --ros-args \\
        -p world_file:=<path to world.yaml> \\
        -p fail_edges:='[hall->commode]' -p fail_mode:=abort
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from washbot_control.world_model import WorldModel

MATCH_RADIUS = 0.35  # goal-to-waypoint matching distance (m)


class FakeNav2Server(Node):

    def __init__(self):
        super().__init__('fake_nav2_server')
        self.declare_parameter('world_file', '')
        self.declare_parameter('speed', 0.35)  # m/s
        self.declare_parameter('tick_rate', 10.0)  # Hz
        self.declare_parameter('fail_locations', [''])
        self.declare_parameter('fail_edges', [''])
        self.declare_parameter('fail_mode', 'abort')  # abort | reject
        self.declare_parameter('fail_times', -1)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)

        self.world = None
        world_file = self.get_parameter('world_file').value
        if world_file:
            self.world = WorldModel.from_yaml(world_file)
            self.x, self.y, _ = self.world.pose_of(self.world.robot_at)
        else:
            self.x = float(self.get_parameter('start_x').value)
            self.y = float(self.get_parameter('start_y').value)

        self._fail_locations = [
            name for name in self.get_parameter('fail_locations').value if name]
        self._fail_edges = []
        for entry in self.get_parameter('fail_edges').value:
            if entry:
                origin, _, destination = entry.partition('->')
                self._fail_edges.append((origin.strip(), destination.strip()))
        self._fail_mode = self.get_parameter('fail_mode').value
        self._fail_times = int(self.get_parameter('fail_times').value)
        self._attempts = {}

        self._pose_pub = self.create_publisher(PoseStamped, '/washbot/sim_pose', 10)
        self._server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self._execute,
            goal_callback=self._on_goal_request,
            cancel_callback=lambda _request: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

        if self._fail_locations or self._fail_edges:
            edges = [f'{a}->{b}' for a, b in self._fail_edges]
            self.get_logger().info(
                f'fault injection: {self._fail_mode} at '
                f'{self._fail_locations + edges} (fail_times={self._fail_times})')
        self.get_logger().info(
            f'fake Nav2 server ready at ({self.x:.2f}, {self.y:.2f})')

    # ------------------------------------------------------- fault matching

    def _match_location(self, x: float, y: float) -> str:
        if self.world is None:
            return ''
        best_name, best_distance = '', MATCH_RADIUS
        for name in self.world.locations:
            wx, wy, _ = self.world.pose_of(name)
            distance = math.hypot(wx - x, wy - y)
            if distance < best_distance:
                best_name, best_distance = name, distance
        return best_name

    def _should_fail(self, x: float, y: float) -> bool:
        destination = self._match_location(x, y)
        if not destination:
            return False

        key = None
        if destination in self._fail_locations:
            key = destination
        else:
            origin = self._match_location(self.x, self.y)
            if origin and (origin, destination) in self._fail_edges:
                key = f'{origin}->{destination}'
        if key is None:
            return False

        attempts = self._attempts.get(key, 0)
        self._attempts[key] = attempts + 1
        if self._fail_times < 0 or attempts < self._fail_times:
            self.get_logger().warning(
                f'injected failure for "{key}" '
                f'(attempt {attempts + 1}, mode={self._fail_mode})')
            return True
        return False

    # -------------------------------------------------------------- serving

    def _on_goal_request(self, goal_request) -> GoalResponse:
        if self._fail_mode == 'reject':
            position = goal_request.pose.pose.position
            if self._should_fail(position.x, position.y):
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request.pose.pose.position
        speed = float(self.get_parameter('speed').value)
        tick = 1.0 / float(self.get_parameter('tick_rate').value)

        fail_mid_drive = (self._fail_mode == 'abort'
                          and self._should_fail(goal.x, goal.y))
        total = math.hypot(goal.x - self.x, goal.y - self.y)
        travelled = 0.0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()

            remaining = math.hypot(goal.x - self.x, goal.y - self.y)
            if remaining <= max(speed * tick, 0.02):
                self.x, self.y = goal.x, goal.y
                self._publish_pose()
                goal_handle.succeed()
                return NavigateToPose.Result()

            # Abort mid-drive once we are ~60% of the way there: far enough to
            # look like the robot really got stuck, close enough to be quick.
            if fail_mid_drive and total > 0 and travelled >= 0.6 * total:
                goal_handle.abort()
                return NavigateToPose.Result()

            step = speed * tick
            self.x += step * (goal.x - self.x) / remaining
            self.y += step * (goal.y - self.y) / remaining
            travelled += step
            self._publish_pose()

            feedback = NavigateToPose.Feedback()
            feedback.current_pose.header.frame_id = 'map'
            feedback.current_pose.header.stamp = self.get_clock().now().to_msg()
            feedback.current_pose.pose.position.x = self.x
            feedback.current_pose.pose.position.y = self.y
            feedback.distance_remaining = remaining
            goal_handle.publish_feedback(feedback)
            time.sleep(tick)

        goal_handle.abort()
        return NavigateToPose.Result()

    def _publish_pose(self) -> None:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.w = 1.0
        self._pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = FakeNav2Server()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
