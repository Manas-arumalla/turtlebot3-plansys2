"""Publish an initial pose estimate for AMCL, then exit.

Replacement for the ad-hoc script the project started with: a proper node
that publishes a few times (AMCL's subscriber may connect late), uses the
node clock for stamps, and exits cleanly.

    ros2 run washbot_control initial_pose_publisher --ros-args \\
        -p x:=-2.0 -p y:=-0.5 -p yaw:=0.0
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

# Publish repeatedly over several seconds: AMCL may still be activating when
# this node starts, and a single early message would be lost.
PUBLISH_COUNT = 10
PUBLISH_PERIOD = 0.8


class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('position_stddev', 0.25)
        self.declare_parameter('yaw_stddev', 0.25)

        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._remaining = PUBLISH_COUNT
        self._timer = self.create_timer(PUBLISH_PERIOD, self._publish)

    def _publish(self) -> None:
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        yaw = float(self.get_parameter('yaw').value)
        position_var = float(self.get_parameter('position_stddev').value) ** 2
        yaw_var = float(self.get_parameter('yaw_stddev').value) ** 2

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = position_var   # x
        msg.pose.covariance[7] = position_var   # y
        msg.pose.covariance[35] = yaw_var       # yaw

        self._publisher.publish(msg)
        self._remaining -= 1
        if self._remaining <= 0:
            self.get_logger().info(
                f'initial pose published: ({x:.2f}, {y:.2f}, yaw {yaw:.2f})')
            self._timer.cancel()
            # Ask the executor to stop spinning us.
            raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
