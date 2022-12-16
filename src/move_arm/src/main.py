#!/usr/bin/env python

import sys
from intera_interface import Limb
import rospy
import numpy as np
import traceback
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped,Pose
from path_planner_original import PathPlanner
from motion_function import MotionFunction
def main():
    # Create a MotionFunction object
    motion_function_object = MotionFunction()

    # Initializing
    motion_function_object.initialize()

    # AR_tag detection
    motion_function_object.analyzing_motion()

    print(motion_function_object.ar_track)

    """
        # Get the glass position 
    glass_1_position_cord = np.array([.720, -0.221 , 0.02]) # get the transformation from AR_tag
    glass_1 = Pose()
    glass_1.position.x = 0.5 #.838
    glass_1.position.y = -0.2 # -0.201
    glass_1.position.z = -0.14

    glass_2_position = np.array([.720 , 0.082, 0.02]) # get the transformation from AR_tag
    glass_2 = Pose()
    glass_2.position.x = 0.5 # .720
    glass_2.position.y = -0.0 #0.053
    glass_2.position.z = -0.14

    pouring_position = np.array([.720 , 0.316, 0.1]) # get the container position
    container = Pose() 
    container.position.x = 0.5 #.836
    container.position.y = 0.02 # 0.082
    container.position.z = 0.02
    """


    glass_1_position = motion_function_object.ar_track["1st_glass"].pose.pose
    glass_2_position = motion_function_object.ar_track["2nd_glass"].pose.pose # get the transformation from AR_tag
    pouring_position = motion_function_object.ar_track["container"].pose.pose # get the container position

    """
    x_val = .45

    glass_1_position.position.x = x_val
    glass_2_position.position.x = x_val
    pouring_position.position.x = x_val
    """
    x_offset = -0.2
    y_offset = -0.02

    print(glass_1_position)
    print(glass_1_position)

    # container biases
    x_b = x_offset
    y_b = -0.06
    z_b = 0.22


    
    # Glass 1
    #motion_function_object.going_and_take_glass(glass_1_position)
    #motion_function_object.move_to_position(pouring_position, x_b, y_b, z_b)

    motion_function_object.move_to_position(glass_1_position, x_offset, y_offset, 0.22)
    
    motion_function_object.move_to_position(glass_1_position, x_offset, y_offset, 0.08)
    motion_function_object.close()
    motion_function_object.move_to_position(glass_1_position, x_offset, y_offset, 0.22)

    motion_function_object.going_to_container_and_pouring(pouring_position, x_b, y_b, z_b)

    motion_function_object.move_to_position(glass_1_position, x_offset, y_offset, 0.22)
    motion_function_object.move_to_position(glass_1_position, x_offset, y_offset, 0.08)
    motion_function_object.open()
    motion_function_object.move_to_position(glass_1_position, x_offset, 0, 0.22)
    
    #motion_function_object.putting_glass_back(glass_1_position)
    
    # Glass 2
    #motion_function_object.going_and_take_glass(glass_2_position)

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



    
   # motion_function_object.putting_glass_back(glass_2_position)
    
    



if __name__ == '__main__':
    rospy.init_node('coktail_node')
    main() # May need to include some inputs like the quantitiy of things to put of each glass
    
