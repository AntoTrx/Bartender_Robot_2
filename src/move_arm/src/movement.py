#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""
import sys
from intera_interface import Limb
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
import object_msg
import object_space_msg

from path_planner import PathPlanner

try:
    from controller import Controller
except ImportError:
    pass

orientation_constraint = OrientationConstraint()
orientation_constraint.link_name = "right_gripper"
orientation_constraint.head.frame_id = "base"
orientation_constraint.y = 1.0
orientation_constraint_y_axis_tolerance = 0.0
orientation_constraint.weight = 1.0

"""
TEST CODE
"""


def move(position):
    planner = PathPlanner("right_arm")

    Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

    controller = Controller(Kp, Ki, Kd, Kw, Limb("right")

    while not rospy.is_shutdown():
        while not rospy.is_shutdown():
            try:
                x, y, z = positon
                goal = PoseStamped()
                goal.header.frame_id = 'base'

                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.position.z = z

                goal.pose.orientation.x = 0
                goal.pose.orientation.y = 1
                goal.pose.orientation.z = 0
                goal.pose.orientation.w = 0

                plan = planner.plan_to_pose(goal, [])

#                input("Press <Enter> to move:")
                if not controller.execute_plan(plan[1]):
                    raise Exception("Execution failed")
            except Exception as e:
                print(e)
            else:
                break


def open_gripper():
    right_gripper.open()
    rospy.sleep(1.0)

def close_gripper():
    right_gripper.close()
    rospy.sleep(1.0)

def pour():
    # rotate gripper to 90-180 degrees


    # rotate gripper back

def main():
    # get cup positions from AR_track_alvar (as object_space_msg)
    """
    object_msg:
        string name
        float32 x
        float32 y
        float32 z

    object_space_msg
        object_msg target
        object_msg a
        object_msg b
    """

    # use positions to move cups



if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
