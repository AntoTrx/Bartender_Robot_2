#!/usr/bin/env python

import sys
from intera_interface import Limb,gripper
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped,Quaternion

from path_planner import PathPlanner



def mat_to_quat (m):
    if not m.shape == (3,3):
        raise TypeError('omega must be a 3x3 matrix')
    
    q = Quaternion()
    q.w=np.sqrt(1 + m[0,0] + m[1,1] + m[2,2])/2
    q.x = (m[2,1] - m[1,2])/q.w
    q.y= (m[0,2] - m[2,0])/q.w
    q.z= (m[1,0] - m[0,1])/q.w
    
    return q



    