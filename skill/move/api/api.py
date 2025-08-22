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

import sys
root_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
if root_dir not in sys.path:
    sys.path.append(root_dir)

print(root_dir)

from DeepEmbody.manager.eaios_decorators import eaios

#TODO memory
mamory = {}

@eaios.skill
def move_to_goal(goal_name:str) -> str:
    """move to a predefined goal position stored in memory
    Args:
        goal_name: the name of the goal to move to, which should be stored in memory
    Returns:
        A string indicating the result of the move operation
    """
    if goal_name in memory.keys():
        return move_to_ab_pos(memory[goal_name])
    else:
        return f"Service setmove_to_goal_gaol response: {False}, message: goal not in memory"


@eaios.skill
def move_to_ab_pos(x, y, yaw) -> str:
    """Move to an absolute position in the map
    Args:
        x: x coordinate of the target point
        y: y coordinate of the target point
        yaw: yaw angle of the target point in radians
    Returns:
        A string indicating the result of the move operation
    """
    #TODO how read dep
    return set_goal(x,y,yaw)

@eaios.skill
def move_to_rel_pos(dx,dy,dyaw) -> str:
    """Move to a relative position based on the current positions
    Args:
        dx: x direction offset
        dy: y direction offset
        dyaw: yaw angle offset in radians
    Returns:
        A string indicating the result of the move operation
    """
    set_goal = dep["move"][1]["set_goal"]
    pos = get_pos()
    return set_goal(pos.x + dx,pos.y + dy,pos.yaw + dyaw)

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
    
#315 pos 28.3 0.1 0
if __name__ == "__main__":
    move_to_ab_pos(-11.7,-6.8,0)