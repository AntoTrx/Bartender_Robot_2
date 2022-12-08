# -*- coding: utf-8 -*-
"""
Created on Wed Nov 30 08:43:18 2022

Serving command

@author: antoi
"""



"""
Create a class? ++ since we can create an object called

Should include all the commands for moving to one part to another

0 - Going to a standard position
A - Going to take to glaas, take it and go back to standar position
B - Going with the glass to the container (/!\ how to identify), and poruing the glass, then going back to a std position
C - Going to the original place for the glass and letting it (need to remember the original place)


"""


import sys
from intera_interface import Limb,gripper
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner


"""

Should use a path planner object

"""

class MotionFunction(object):


    def __init__(self):

        self.planner = PathPlanner('rigth_arm')

        self.right_gripper = gripper.Gripper('right_gripper')

        #Create a path constraint for the arm
        #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
        self.orien_const = OrientationConstraint()
        self.orien_const.link_name = "right_gripper";
        self.orien_const.header.frame_id = "base";
        self.orien_const.orientation.y = -1.0;
        self.orien_const.absolute_x_axis_tolerance = 0.1;
        self.absolute_y_axis_tolerance = 0.1;
        self.orien_const.absolute_z_axis_tolerance = 0.1;
        self.orien_const.weight = 1.0;

        self.right_gripper.calibrate()
        rospy.sleep(2.0)







    def initialize(self):
        """
        A function to calibrate the gripper and put the robot in the standard position

        """
        # Calibrate the gripper (other commands won't work unless you do this first)

        self.going_to_standard_position()




    def going_to_standard_position(self):
        """
        Going to the std posiiton (Phase 0)
        Can be used inside the other position /!\ with the gripper

        """

        while not rospy.is_shutdown():
            try :
                x,y,z = 0,0,0

                goal = PoseStamped()
                goal.header.frame_id = "base"

                #x, y, and z position
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.position.z = z

                #Orientation as a quaternion
                goal.pose.orientation.x = 0.0
                goal.pose.orientation.y = -1.0
                goal.pose.orientation.z = 0.0
                goal.pose.orientation.w = 0.0

                plan = self.planner.plan_to_pose(goal, [self.orien_const])
                input("Press <Enter> to move the right arm to goal pose 1: ")

                if not self.planner.execute_plan(plan[1]):
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break

        return True





    def going_and_take_glass(self,glass_position):
        """
        We do phase A

        """


        # Going to the glass,
        while not rospy.is_shutdown():
            try :
                x,y,z = glass_position

                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"

                #x, y, and z position
                goal_1.pose.position.x = x
                goal_1.pose.position.y = y
                goal_1.pose.position.z = z

                #Orientation as a quaternion
                goal_1.pose.orientation.x = 0.0
                goal_1.pose.orientation.y = -1.0
                goal_1.pose.orientation.z = 0.0
                goal_1.pose.orientation.w = 0.0

                plan_1 = self.planner.plan_to_pose(goal_1, [self.orien_const])
                input("Press <Enter> to move the right arm to goal pose 1: ")

                if not self.planner.execute_plan(plan_1[1]):
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break


        # Closing the gripper
        self.right_gripper.close()
        rospy.sleep(1.0)


        ### Going to standard position
        self.going_to_standard_position()



        return True









    def going_to_container_and_pouring(self, container_position):
        """
        We do phase B

        """
        ### Going to the container
        while not rospy.is_shutdown():
            try :
                x,y,z = container_position #Container position

                goal_2 = PoseStamped()
                goal_2.header.frame_id = "base"

                #x, y, and z position
                goal_2.pose.position.x = x
                goal_2.pose.position.y = y
                goal_2.pose.position.z = z

                #Orientation as a quaternion
                goal_2.pose.orientation.x = 0.0
                goal_2.pose.orientation.y = -1.0
                goal_2.pose.orientation.z = 0.0
                goal_2.pose.orientation.w = 0.0

                plan_2 = self.planner.plan_to_pose(goal_2, [self.orien_const])
                input("Press <Enter> to move the right arm to goal pose 1: ")

                if not self.planner.execute_plan(plan_2[1]):
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break

        ### Pouring
                """
                Acting on the joint directly


                """

                " first Motion = going down"
                " Then waiting"
                " Then going up"


        ### Going to standard position
        self.going_to_standard_position()

        return True



    def putting_glass_back(self,glass_position):
        """
        Putting again the glass at the original position
        """
        ### Going to the initial location
        while not rospy.is_shutdown():
            try :
                x,y,z = glass_position # Going to the initial configuration

                goal_3 = PoseStamped()
                goal_3.header.frame_id = "base"

                #x, y, and z position
                goal_3.pose.position.x = x
                goal_3.pose.position.y = y
                goal_3.pose.position.z = z

                #Orientation as a quaternion
                goal_3.pose.orientation.x = 0.0
                goal_3.pose.orientation.y = -1.0
                goal_3.pose.orientation.z = 0.0
                goal_3.pose.orientation.w = 0.0

                plan_3 = self.planner.plan_to_pose(goal_3, [self.orien_const])
                input("Press <Enter> to move the right arm to goal pose 1: ")

                if not self.planner.execute_plan(plan_3[1]):
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break

        ### Opening the gripper
        self.right_gripper.open()
        rospy.sleep(1.0)
        print('Done!')

        ### Going to standard position
        self.going_to_standard_position()

        return True

    """
    Simpler Implementation??
    """

    def move(self, position):
        return True

    def close_gripper(self):
        return True

    def open_gripper(self):
        return True

    def pour(self):
        return True
