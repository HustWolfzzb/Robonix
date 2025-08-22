#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from mcp.server.fastmcp import FastMCP

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import Range
import sys

from DeepEmbody.manager.eaios_decorators import eaios

class NavWithUltrasonicSafety(Node):
    def __init__(self,safety_threshold=0.5):
        super().__init__('nav_with_ultrasonic_safety')
        self.navigator = BasicNavigator()
        self.safety_threshold = safety_threshold
        self.cancelled = False

        # create subscription to ultrasonic sensor
        self.create_subscription(Range, '/ultrasonic/sensor0_front', self.range_callback, 10)


    def set_goal(self, x, y, yaw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.orientation.z = float(yaw)  # 这里可以优化为真实 yaw 转 quaternion
        goal_pose.pose.orientation.w = 1.0

        self.get_logger().info('Going to goal pose...')
        self.navigator.goThroughPoses([goal_pose])

        # monitor the task
        while not self.navigator.isTaskComplete() and not self.cancelled:
            rclpy.spin_once(self, timeout_sec=0.5)

        if self.cancelled:
            self.get_logger().warn('Navigation cancelled due to ultrasonic obstacle.')
            return False
        else:
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Goal succeeded!')
                return True
            else:
                self.get_logger().warn('Goal failed!')
                return False

    def range_callback(self, msg: Range):
        distance = msg.range
        self.get_logger().info(f'Ultrasonic distance: {distance:.2f} meters')

        if distance < self.safety_threshold and not self.cancelled:
            self.get_logger().warn('Obstacle too close! Cancelling navigation...')
            self.navigator.cancelTask()
            self.cancelled = True

@eaios.cap
def nv_test():
    import time
    if int(time.time())%2 == 0:
        func = eaios.get_plugin("navigation2","ros2_navigation")
    else:
        func = eaios.get_plugin("navigation2","simple_navigation")
    # res = func()
    print("lhe debug in cap test nv res", func, id(func))
    # return res
    return func()

@eaios.cap
def set_goal(x, y, yaw) -> str:
    """set navigation goal
    Args:
        x: x coordinate of the target point
        y: y coordinate of the target point
        yaw: yaw angle of the target point in radians
    """
    # rclpy.init()
    import yaml
    plugin_name = "simple_navigation"
    config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "description.yml")
    with open(config_path, "r") as f:
        description_data = yaml.safe_load(f)
        plugin_name = description_data.get("plugins", [])[0]  # 获取第一个插件名称
    if plugin_name == "ros2_navigation":
        func = eaios.get_plugin("navigation2","ros2_navigation")
    else:
        func = eaios.get_plugin("navigation2","simple_navigation")
    res = func(x,y,yaw)
    func_status = f"Service set_gaol response: {res}"
    return func_status

@eaios.cap
def stop_goal() -> str:
    """Stop the current navigation goal.
    Args:
        None
    """
    rclpy.init()
    nv_controller.cancelled = True
    func_status = f"Service stop response: {True}"
    rclpy.shutdown()
    return func_status

def test():
    rclpy.init()
    node = NodeController()

    # ros2 service list
    # ros2 service type /get_count
    # ros2 service call get_count std_srvs/srv/Trigger

    req = Trigger.Request()
    res = node.call_service('get_count', req)
    print(f"Service get_count response: {res.success}, message: {res.message}")

    req = Trigger.Request()
    res = node.call_service('modify_name', req)
    print(f"Service modify_name response: {res.success}, message: {res.message}")

    req = Trigger.Request()
    res = node.call_service('shutdown_node', req)
    print(f"Service shutdown_node response: {res.success}, message: {res.message}")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    mcp.run(transport='stdio')