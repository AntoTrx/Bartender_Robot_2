#!/usr/bin/env python

"""
This Python file is a class that gather the different necessary paths and tasks, to make the motion part easier.
    
"""


import sys
from intera_interface import gripper
import rospy
import numpy as np
import traceback
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
from path_planner_original import PathPlanner
from controller import Controller
import math_functions
from tf.transformations import quaternion_from_euler
from ar_track_alvar_msgs.msg import AlvarMarkers



"""

Should use a path planner object

"""
# Python Class MotionFunction
class MotionFunction(object):
    
    def __init__(self):
        
        # Create a PathPlanner Object
        self.planner = PathPlanner('right_arm')

        # Create a Gripper object to use Sawyer's gripper
        self.right_gripper = gripper.Gripper('right')
    
        # Dictionary that conserves the different positions from the different objects    
        self.ar_track = {"1st_glass" : 0, "2nd_glass"  : 0, "container": 0}
        

    
     
    def initialize(self):
        """
        This function calibrates the gripper and prepares the arm for what is to come
        """
    
        
        
        # Calibrate the gripper (other commands won't work unless you do this first)
        self.right_gripper.calibrate()
        rospy.sleep(0.5)

        self.right_gripper.open()
        rospy.sleep(0.5)

        # We use the function move_to_neutral of the object Limb (usful to call sawyer moitons' functions)
        self.planner._limb.move_to_neutral()

                
        
    def analyzing_motion(self):
        """
        This function does a motion from right to left (in the arm's frame)in order to get all the positions of the different elements
        """

        plan = {'right_j0': -1.1822275390625,'right_j1': 0.501970703125,'right_j2': -1.3819697265625,
            'right_j3':-1.1907275390625,'right_j4': 1.0726474609375, 
                'right_j5': 0.0539248046875, 'right_j6': 1.7883544921875}

        velocities = {'right_j0': 0.2,'right_j1': 0,'right_j2': 0,'right_j3':0,'right_j4': 0, 'right_j5': 0, 'right_j6': 0}
        velocities_0 = {'right_j0': 0,'right_j1': 0,'right_j2': 0,'right_j3':0,'right_j4': 0, 'right_j5': 0, 'right_j6': 0}
        
        # Executing the plan
        if not self.planner.execute_plan(plan): 
            raise Exception("Execution failed")

        rospy.sleep(1.0)

        # Initialize the number of steps
        i = 0
        while ((self.ar_track["1st_glass"] == 0) or (self.ar_track["2nd_glass"] == 0) or (self.ar_track["container"] == 0) ) and (i<7):
            # # Wait for the ar_pose_marker topic to become available
            
            rospy.loginfo("Waiting for ar_pose_marker topic...")
            rospy.wait_for_message('ar_pose_marker', AlvarMarkers)
            

            # Subscribe to the ar_pose_marker topic and execute the detect_ar function
            rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.detect_ar)

            # We move the arm around the right_j0 joint to get all the Ar tags
            self.planner._limb.set_joint_velocities(velocities)
            
            print(self.ar_track)

        self.planner._limb.set_joint_velocities(velocities_0)
        print("Analyzing...")

        rospy.sleep(1.0)

        self.planner._limb.move_to_neutral()
        self.going_to_standard_position()




    def detect_ar(self,msg):
        """
        This function dis called when it recieves the AlvarMarkers message from the ar_pose_marker topic, which is a list
        of AlvarMarker. Each marker has an ID and a PoseStamped list. We use a tf function from tf package to change the coordinate
        frame
        """

        for elt in msg.markers:

            pose_wrt_camera = PoseStamped()
            pose_wrt_camera.header.frame_id = "right_hand_camera" #Camera_id

            pose_wrt_camera.pose = elt.pose

            # We associate the right AR tag to the right object
            
            if self.ar_track["1st_glass"] == 0 and elt.id == 6 :
                self.ar_track["1st_glass"] = pose_wrt_camera
            elif self.ar_track["2nd_glass"] == 0 and elt.id == 17 : 
                self.ar_track["2nd_glass"] = pose_wrt_camera
            elif self.ar_track["container"] == 0 and elt.id == 0 :
                self.ar_track["container"] = pose_wrt_camera
            else:
                break
            


    def going_to_standard_position(self):
        """
        Going to a standard position
        """
        while not rospy.is_shutdown():
            try :
                x,y,z = 0.6,0.2,0.2
                
                goal = PoseStamped()
                goal.header.frame_id = "base"
            
                #x, y, and z position                
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.position.z = z
            
                #Orientation as a quaternion
                goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w = quaternion_from_euler(-np.pi,-np.pi/2,0)
                
                plan = self.planner.plan_to_pose(goal)
                
                if not self.planner.execute_plan(plan): 
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break

    

    def open(self):
        """
        Open the gripper
        """
        self.right_gripper.open()
        rospy.sleep(0.5)

    def close(self):
        """
        Close the gripper
        """
        self.right_gripper.close()
        rospy.sleep(0.5)
        
    def move_to_position(self, glass_position, x_bias=0.0, y_bias=0.0, z_bias=0.0):
        """
        Move to a defined position
        """
        while not rospy.is_shutdown():
            try :
                x,y,z = glass_position.position.x,glass_position.position.y,glass_position.position.z
                
                goal_0 = PoseStamped()
                goal_0.header.frame_id = "base"
            
                #x, y, and z position
                goal_0.pose.position.x = x + x_bias
                goal_0.pose.position.y = y + y_bias
                goal_0.pose.position.z = z + z_bias
            
                #Orientation as a quaternion
                # goal_0.pose.orientation.x,goal_0.pose.orientation.y,goal_0.pose.orientation.z,goal_0.pose.orientation.w = quaternion_from_euler(np.pi,0,np.pi)
                goal_0.pose.orientation.x,goal_0.pose.orientation.y,goal_0.pose.orientation.z,goal_0.pose.orientation.w = quaternion_from_euler(-np.pi,-np.pi/2,0)
 
                
                plan_0 = self.planner.plan_to_pose(goal_0, [self.orien_const])
                # input("Press <Enter> to move the right arm glass: ")
                
                if not self.planner.execute_plan(plan_0): 
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break
        rospy.sleep(0.25)

    
    
    def going_and_take_glass(self,glass_position):
        """
        Function that goes to the defined glass and take it
        
        """
        
        
        # Going to the glass,
        # step 1 : going above:
        while not rospy.is_shutdown():
            try :
                x,y,z = glass_position.position.x,glass_position.position.y,glass_position.position.z
                
                goal_0 = PoseStamped()
                goal_0.header.frame_id = "base"
            
                #x, y, and z position
                goal_0.pose.position.x = x
                goal_0.pose.position.y = y
                goal_0.pose.position.z = z 
            
                #Orientation as a quaternion
                goal_0.pose.orientation.x,goal_0.pose.orientation.y,goal_0.pose.orientation.z,goal_0.pose.orientation.w = quaternion_from_euler(-np.pi,-np.pi/2,0)
 
                
                plan_0 = self.planner.plan_to_pose(goal_0, [self.orien_const])
                
                if not self.planner.execute_plan(plan_0): 
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break


        # Step 2 Going to the glass
        while not rospy.is_shutdown():
            try :
            
                
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"
            
                #x, y, and z position
                goal_1.pose.position.x = x
                goal_1.pose.position.y = y
                goal_1.pose.position.z = z
            
                #Orientation as a quaternion
                goal_1.pose.orientation.x,goal_1.pose.orientation.y,goal_1.pose.orientation.z,goal_1.pose.orientation.w = quaternion_from_euler(-np.pi,-np.pi/2,0)
                
                plan_1 = self.planner.plan_to_pose(goal_1, [self.orien_const])
                
                if not self.planner.execute_plan(plan_1): 
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break
        
        rospy.sleep(0.5)

        # Closing the gripper
        print("Closing Gripper")
        self.right_gripper.close()
        rospy.sleep(0.5)
            
            
        # Going to standard position
        self.going_to_standard_position()
            
      
        
    def going_to_container_and_pouring(self, container_position, x_bias=0.0, y_bias=0.0, z_bias=0.0):
        """
        Function that goes to container with the glass and pour the content
        
        """
        x,y,z = container_position.position.x,container_position.position.y,container_position.position.z
        x += x_bias
        y += y_bias
        z += z_bias
        
            
        plan_45 = None 


        while not rospy.is_shutdown():
            try :
                
                goal_3 = PoseStamped()
                goal_3.header.frame_id = "base"
            
                #x, y, and z position
                goal_3.pose.position.x = x 
                goal_3.pose.position.y = y
                goal_3.pose.position.z = z
            
                #Orientation as a quaternion
                goal_3.pose.orientation.x,goal_3.pose.orientation.y,goal_3.pose.orientation.z,goal_3.pose.orientation.w = quaternion_from_euler(-np.pi,-np.pi/2,0)
                
                plan_3 = self.planner.plan_to_pose(goal_3, [self.orien_const])
                plan_45 = plan_3
                
                if not self.planner.execute_plan(plan_3): 
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break
         
        # Second step, pouring the glass
        # Rotation auround the joint axis
        while not rospy.is_shutdown():
            try :
                
                plan_4 = plan_45.copy()
                print(plan_4)
                plan_4['right_j6'] -= 1.75

                if not self.planner.execute_plan(plan_4): 
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break
            
        # Break to be sure that our content is going down   
        rospy.sleep(1.0)
        
        # Third step, going up again
        while not rospy.is_shutdown():
            try :
                
                plan_5 = plan_45
                
                if not self.planner.execute_plan(plan_5): 
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break           
        
        rospy.sleep(0.5)
    
            
        
    def putting_glass_back(self,glass_position):
        """
        Putting again the glass at the original position
        """
        ### Going to the initial location
        while not rospy.is_shutdown():
            try :
                x,y,z = glass_position.position.x,glass_position.position.y,glass_position.position.z # Going to the initial configuration
                
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"
            
                #x, y, and z position
                goal_1.pose.position.x = x
                goal_1.pose.position.y = y
                goal_1.pose.position.z = z
            
                #Orientation as a quaternion
                goal_1.pose.orientation.x,goal_1.pose.orientation.y,goal_1.pose.orientation.z,goal_1.pose.orientation.w = quaternion_from_euler(-np.pi,-np.pi/2,0)
                
                plan_1 = self.planner.plan_to_pose(goal_1)
                
                if not self.planner.execute_plan(plan_1): 
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break
            
        # Opening the gripper
        rospy.sleep(.5)

        print("Opening Gripper")
        self.right_gripper.open()
        rospy.sleep(.5)      
        
        # Going to standard position
        self.going_to_standard_position()
        
        return True   
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            