#!/usr/bin/env python
"""
Path Planner Class for Lab 7, we have tuned in order to use both Sawyer_kinematics' functions
and Intera_interface motions' functions.
"""

import sys
import rospy
import moveit_commander
from intera_interface import Limb
from sawyer_kinematics.sawyer_kinematics.sawyer_kinematics import Sawyer

class PathPlanner(object):
    """
    We make this a class rather than a script because it bundles up 
    all the code relating to planning in a nice way thus, we can
    easily use the code in different places. This is a staple of
    good object-oriented programming

    """

    def __init__(self, group_name):
        """
        Constructor.

        Inputs:
        group_name: the name of the move_group.
            For Baxter, this would be 'left_arm' or 'right_arm'
            For Sawyer, this would be 'right_arm'
        """
        # Create a Saywer objet, in order to use Intera's IK solver
        self._robot_bis = Sawyer()

        # Create a Limb objet, in order to use Intera's IK solver
        self._limb = Limb('right')

        # Create a plan, that is a dictionary of the different angles with a value on it
        self._plan = self._limb.joint_angles()

        # If the node is shutdown, call this function    
        rospy.on_shutdown(self.shutdown)

        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.sleep(0.5)

    def shutdown(self):
        """
        Code to run on shutdown. This is good practice for safety

        Currently deletes the object's MoveGroup, so that further commands will do nothing
        """
        # self._group = None
        rospy.is_shutdown() 
        rospy.loginfo("Stopping Path Planner")

    def plan_to_pose(self, target, orientation_constraints):
        """
        Generates a plan given an end effector pose subject to orientation constraints

        Inputs:
        target: A geometry_msgs/PoseStamped message containing the end effector pose goal
        orientation_constraints: A list of moveit_msgs/OrientationConstraint messages

        Outputs:
        path: A moveit_msgs/RobotTrajectory path
        """

        plan = self._robot_bis.Inverse_Kinematics(self._limb, target)

        self._plan = plan

        return plan

    def execute_plan(self, plan):
        """
        Uses the robot's built-in controllers to execute a plan

        Inputs:
        plan: a moveit_msgs/RobotTrajectory plan
        """
        self._limb.move_to_joint_positions(plan)
        print('execute_plan')

        return True

