# -*- coding: utf-8 -*-
"""
Created on Wed Nov 30 08:38:41 2022

@author: antoi
"""
import sys
from intera_interface import Limb
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner

from motion_function import MotionFunction




def main():
    
    # Create a MotionFunction object
    motion_function_object = MotionFunction()
    
    # Get the glass position 
    glass_1_position = np.array([.5, .5, .5]) # get the transformation from AR_tag
    glass_2_position = np.array([.25, .25, .25]) # get the transformation from AR_tag
    container_position = np.array([1, 1, 1]) # get the container position
    
    # Glass 1
    motion_function_object.going_and_take_glass(glass_1_position)

    motion_function_object.going_to_container_and_pouring(container_position)
    
    motion_function_object.putting_glass_back(glass_1_position)
    
    # Glass 2
    motion_function_object.going_and_take_glass(glass_2_position)

    motion_function_object.going_to_container_and_pouring(container_position)
    
    motion_function_object.putting_glass_back(glass_2_position)
    
    



if __name__ == '__main__':
    rospy.init_node('coktail_node')
    main() # May need to include some inputs like the quantitiy of things to put of each glass
