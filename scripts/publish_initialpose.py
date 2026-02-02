#!/usr/bin/env python3
# publish_initialpose.py
# Usage: python3 publish_initialpose.py X Y YAW_SECONDS_AHEAD
# Example: python3 publish_initialpose.py 0.0 0.0 0.0

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from builtin_interfaces.msg import Time
import sys
import math
import time

def yaw_to_quaternion(yaw):
    import math
    qz = math.sin(yaw/2.0)
    qw = math.cos(yaw/2.0)
    return (0.0, 0.0, qz, qw)

class InitialPosePublisher(Node):
    def __init__(self, x, y, yaw, ahead_seconds=3):
        super().__init__('initialpose_pub')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.get_logger().info(f'Waiting for /clock to appear...')
        # Wait until /clock publishes at least once (sim time)
        cli = self.create_subscription_msg(
            'builtin_interfaces/msg/Time', '/clock', lambda msg: None)
    def create_subscription_msg(self, _type, topic, cb):
        # simple wait-for-topic helper
        sub = self.create_subscription(_type, topic, lambda msg: None, 1)
        timeout = time.time() + 10.0
        while rclpy.ok():
            # check publishers count
            info = self.get_publisher_count(topic)
            # easier: wait until /clock topic has data by sleeping a bit
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() > timeout:
                break
        return sub

def publish_once(x,y,yaw,ahead=3):
    rclpy.init()
    node = rclpy.create_node('initialpose_pub_node')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    # read /clock once
    clock_val = 0
    try:
        # Try to get sim time by echoing /clock using rclpy wait (simple sleep fallback)
        # Sleep briefly to allow /clock to advance
        for _ in range(6):
            rclpy.spin_once(node, timeout_sec=0.2)
        # fallback: use current system time secs if /clock missing
    except:
        pass
    import time as _t
    # set stamp ahead seconds
    stamp_sec = int(_t.time()) + ahead
    qx,qy,qz,qw = 0.0,0.0, *yaw_to_quaternion(yaw)[2:]
    msg = PoseWithCovarianceStamped()
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = 0
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = float(qz)
    msg.pose.pose.orientation.w = float(qw)
    cov = [0.05]*36
    msg.pose.covariance = cov
    node.get_logger().info(f'Publishing initialpose at sec={stamp_sec} frame=map x={x} y={y} yaw={yaw}')
    pub.publish(msg)
    # wait a bit so message is sent
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: publish_initialpose.py X Y YAW_in_rad")
    else:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        yaw = float(sys.argv[3])
        publish_once(x,y,yaw, ahead=3)
