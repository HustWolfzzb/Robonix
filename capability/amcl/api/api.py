#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from mcp.server.fastmcp import FastMCP

from tf_transformations import euler_from_quaternion

from DeepEmbody.manager.eaios_decorators import eaios


class AmclPoseGetter(Node):
    def __init__(self):
        super().__init__('amcl_pose_getter')
        self.pose = None
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        self.pose = msg
        self.get_logger().info("Got AMCL pose.")
        self.destroy_subscription(self.subscription)

@eaios.cap
def get_pose(timeout_sec=2.0):
    """get the current AMCL pose
    Args:
        timeout_sec: timeout in seconds to wait for the pose
    Returns:
        A tuple of (x, y, yaw) if pose is received, otherwise None
    """
    rclpy.init()
    node = AmclPoseGetter()

    # Wait for the pose to be received or timeout
    end_time = node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
    while rclpy.ok() and node.pose is None and node.get_clock().now().nanoseconds < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    result = None
    if node.pose:
        pos = node.pose.pose.pose.position
        ori = node.pose.pose.pose.orientation
        yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]
        result = (pos.x, pos.y, yaw)

    node.destroy_node()
    rclpy.shutdown()
    return result
