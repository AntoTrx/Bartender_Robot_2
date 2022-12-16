#!/usr/bin/env python
"""
Main python file for Bartender Robot vision

Take in an image, and use image to find positions of AR Tags
"""

import rospy
import message_filters
import ros_numpy
import tf

import object_msg
import object_space_msg

def main():
    rospy.init_node('targets')



    r.rospy.Rate(1000)

    while not rospy.is_shutdown():
    # get camera data

    # get AR tag data from AR_TAG_ALVAR
