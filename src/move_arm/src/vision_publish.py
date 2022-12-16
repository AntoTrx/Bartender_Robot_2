#!/usr/bin/env python

import sys
import rospy
import cv_bridge
import cv2
import numpy as numpy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, Range
from move_arm.srv import *

#print('vision publisher')

class coordinate(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

def call_find_cups():
    print('call find_cups')
    request = targetRequest()
    request.data = 0
    rospy.wait_for_service('find_cups')
    get_positions = rospy.ServiceProxy('find_cups', target)
    
    while True:
        print('attempting...')
        try:
            positions = get_positions.call(request)
            print(positions)
            return
        except rospy.ServiceException as exc:
            print("Service did not process request...")

def callback(data):
    return

if __name__ == '__main__':
    call_find_cups()
"""
    try:
        coord = coordinate()
        rospy.init_node('detect_tags')
"""
