"""Record a robot's pose stream to CSV for post-mission trajectory analysis.

Used to produce the executed-path figures in the documentation. Works with
the simulated server's pose topic as well as AMCL and odometry on a real
robot / Gazebo:

    # simulated missions
    ros2 run washbot_control pose_recorder --ros-args \\
        -p topic:=/washbot/sim_pose -p output:=/tmp/mission_path.csv

    # Gazebo / real robot (AMCL estimate)
    ros2 run washbot_control pose_recorder --ros-args \\
        -p topic:=/amcl_pose -p msg_type:=pose_with_covariance \\
        -p output:=/tmp/mission_path.csv

The CSV has columns ``t,x,y`` with ``t`` relative to the first sample.
Rendering is done offline with ``washbot_plan render-world --path-csv``.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

_MSG_TYPES = {
    'pose_stamped': (PoseStamped, lambda m: (m.header.stamp, m.pose.position)),
    'pose_with_covariance': (PoseWithCovarianceStamped,
                             lambda m: (m.header.stamp, m.pose.pose.position)),
    'odometry': (Odometry, lambda m: (m.header.stamp, m.pose.pose.position)),
}


class PoseRecorder(Node):

    def __init__(self):
        super().__init__('pose_recorder')
        self.declare_parameter('topic', '/washbot/sim_pose')
        self.declare_parameter('msg_type', 'pose_stamped')
        self.declare_parameter('output', '/tmp/mission_path.csv')

        topic = self.get_parameter('topic').value
        msg_type = self.get_parameter('msg_type').value
        if msg_type not in _MSG_TYPES:
            raise RuntimeError(
                f'unknown msg_type "{msg_type}" '
                f'(expected one of: {", ".join(_MSG_TYPES)})')
        message_class, self._extract = _MSG_TYPES[msg_type]

        output = self.get_parameter('output').value
        self._handle = open(output, 'w', encoding='utf-8')
        self._handle.write('t,x,y\n')
        self._start = None
        self._samples = 0
        self.create_subscription(message_class, topic, self._on_message, 50)
        self.get_logger().info(f'recording {topic} ({msg_type}) -> {output}')

    def _on_message(self, msg) -> None:
        stamp, position = self._extract(msg)
        seconds = stamp.sec + stamp.nanosec * 1e-9
        if self._start is None:
            self._start = seconds
        self._handle.write(f'{seconds - self._start:.3f},'
                           f'{position.x:.4f},{position.y:.4f}\n')
        self._samples += 1

    def close(self) -> None:
        self._handle.flush()
        self._handle.close()
        self.get_logger().info(f'saved {self._samples} samples')


def main(args=None):
    rclpy.init(args=args)
    node = PoseRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
