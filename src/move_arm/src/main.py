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

    """
    TEMPORARY """
    """ USER INPUT ORDER """

    ingredients_list = [glass_1_position, glass_2_position]

    print('|===============================|')
    print('| ** Make-To-Order Cocktails ** |')
    print('| Mixers:                       |')
    print('| 1. Cup_1                      |')
    print('| 2. Cup_2                      |')
    print('|===============================|')
    print('Input ingredient 1:')
    order_1 = eval(input()) - 1
    print('Input ingredient 2 or 0 for none:')
    order_2 = eval(input()) - 1

    motion_function_object.going_and_take_glass(ingredients_list[order_1])

    motion_function_object.going_to_container_and_pouring(container_position)

    motion_function_object.putting_glass_back(ingredients_list[order_1])

    # Glass 2
    if order_2 > 0:
        motion_function_object.going_and_take_glass(ingredients_list[order_2])

        motion_function_object.going_to_container_and_pouring(container_position)

        motion_function_object.putting_glass_back(ingredients_list[order_2])

    """
    TEMPORARY """


    """
    Actions (possibly easier)
    - move(ingredient_1_position)
    - close_gripper()
    - move(target_cup_position)
    - rotate_gripper()
    - move(ingredient_1_position)
    - open_gripper()

    - move(default_position)

    - move(ingredient_2_position)
    - close_gripper()
    - move(target_cup_position)
    - rotate_gripper()
    - move(ingredient_2_position)
    - open_gripper()

    -move(default_position)
    """
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
