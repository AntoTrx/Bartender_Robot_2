#!/usr/bin/env python

"""
Main function that is the link between the different functions. It is this function that we callign to launch the zhole process

"""

### We import several tools

import sys
import rospy
import numpy as np
import traceback
from path_planner_original import PathPlanner
from motion_function import MotionFunction

### Function mian
def main():
    # Create a MotionFunction object
    motion_function_object = MotionFunction()

    # Initializing
    motion_function_object.initialize()

    # AR_tag detection
    motion_function_object.analyzing_motion()

    # We associate the position to the Ar tag to the right glass
    glass_1_position = motion_function_object.ar_track["1st_glass"].pose.pose
    glass_2_position = motion_function_object.ar_track["2nd_glass"].pose.pose # get the transformation from AR_tag
    pouring_position = motion_function_object.ar_track["container"].pose.pose # get the container position

    # We create some offset values in order to improve the behavior of the arm
    x_offset = -0.2
    y_offset = -0.02

    # container biases
    x_b = x_offset
    y_b = -0.06
    z_b = 0.22


    
    # Glass 1
    motion_function_object.move_to_position(glass_1_position, x_offset, y_offset, 0.22)
    motion_function_object.move_to_position(glass_1_position, x_offset, y_offset, 0.08)
    motion_function_object.close()
    motion_function_object.move_to_position(glass_1_position, x_offset, y_offset, 0.22)

    motion_function_object.going_to_container_and_pouring(pouring_position, x_b, y_b, z_b)

    motion_function_object.move_to_position(glass_1_position, x_offset, y_offset, 0.22)
    motion_function_object.move_to_position(glass_1_position, x_offset, y_offset, 0.08)
    motion_function_object.open()
    motion_function_object.move_to_position(glass_1_position, x_offset, 0, 0.22)
    
    
    # Glass 2
    motion_function_object.move_to_position(glass_2_position, x_offset, y_offset, 0.22)
    motion_function_object.move_to_position(glass_2_position, x_offset, y_offset, 0.08)
    motion_function_object.close()
    motion_function_object.move_to_position(glass_2_position, x_offset, y_offset, 0.22)

    motion_function_object.going_to_container_and_pouring(pouring_position, x_b, y_b, z_b)

    motion_function_object.move_to_position(glass_2_position, x_offset, y_offset, 0.22)
    motion_function_object.move_to_position(glass_2_position, x_offset, y_offset, 0.08)
    motion_function_object.open()
    motion_function_object.move_to_position(glass_2_position, x_offset, y_offset, 0.22)

    
    motion_function_object.going_to_standard_position()
    motion_function_object.planner._limb.move_to_neutral()



    

    



if __name__ == '__main__':
    rospy.init_node('coktail_node')
    main()
    
