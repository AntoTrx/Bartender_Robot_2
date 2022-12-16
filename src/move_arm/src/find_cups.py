#!/usr/bin/env python

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose
from move_arm.srv import *
import rospy

class ar_tracker():
    def __init__(self, id = 0):
        """
        serving cup 0
        ingredient1 6
        ingredient2 17
        """

        self._id = id
        self._position = Pose()
        self._positions = {}
        self._found = False

    def callback(self, data):
        ar_marker = data
        markers = ar_marker.markers

        for marker in markers:
            self._positions[marker.id] = marker.position
            if marker.id == self._id:
                self._position = marker.pose.pose
                self._found = True
                rospy.Service('find_cups', target, self.get_target).shutdown()
                return


    def get_target(self, req):
        #print("get target!!!!!!!!!!!")
        rospy.wait_for_message("/ar_pose_marker", AlvarMarkers)
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)

        #print(self._positions+ "positions")
        return targetResponse(self._position)

    def start_target_server(self):
        rospy.init_node('get_target_server')
        s = rospy.Service('find_cups', target, self.get_target)
        rospy.sleep(1)

    def printout(self):
        if self._found:
            print(self._position)
        else:
            print("Tag position not found.")


if __name__ == "__main__":
    #print("###########################init\n|||||||||||||||||||||||||||||||\n\n")
    print('START FIND CUPS')
    v = ar_tracker(0)
    v.start_target_server()
    print(v._positions)
    v.printout()
