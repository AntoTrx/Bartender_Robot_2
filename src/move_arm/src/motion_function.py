#!/usr/bin/env python

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
from geometry_msgs.msg import PoseStamped,Twist

from path_planner_original import PathPlanner
from controller import Controller

import math_functions
from tf.transformations import quaternion_from_euler

from ar_track_alvar_msgs.msg import AlvarMarkers

import tf


"""

Should use a path planner object

"""

class MotionFunction(object):
    
    def __init__(self):
        
        
        self.planner = PathPlanner('right_arm')
        self.right_gripper = gripper.Gripper('right')
        
    
        
        #Create a path constraint for the arm
        #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
        self.orien_const = OrientationConstraint()




        # Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        # Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        # Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        # Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        # self.controller = Controller(Kp, Ki, Kd, Kw,self.limb)
        
        self.ar_track = {"1st_glass" : 0, "2nd_glass"  : 0, "container": 0}
        
        
        
            
        
        

        
    
     
    def initialize(self):
        """
        A function to calibrate the gripper and put the robot in the standard position
        
        """
        # Doing the left to right position in order to get all the positions
        
        
        
        x,y,z= 0.5,-0.2,0.2
        
        
        # Calibrate the gripper (other commands won't work unless you do this first)
        
        """ 
        try:
            self.orien_const.link_name = "right_gripper"
            self.orien_const.header.frame_id = "base"

            orientation_const = (-np.pi,0,0)
            self.orien_const.orientation.x,self.orien_const.orientation.y,self.orien_const.orientation.z,self.orien_const.orientation.w    = quaternion_from_euler(np.pi,0,np.pi)
            self.orien_const.absolute_x_axis_tolerance = 0.1
            self.absolute_y_axis_tolerance = 0.1
            self.orien_const.absolute_z_axis_tolerance = 0.1
            self.orien_const.weight = 0.1
            
            self.right_gripper.calibrate()
            
        except Exception:
            self.orien_const.link_name = "reference/right_gripper"
            self.orien_const.header.frame_id = "base"

            orientation_const = (-np.pi,0,0)
            self.orien_const.orientation.x,self.orien_const.orientation.y,self.orien_const.orientation.z,self.orien_const.orientation.w    = quaternion_from_euler(np.pi,0,np.pi)
            self.orien_const.absolute_x_axis_tolerance = 0.1
            self.absolute_y_axis_tolerance = 0.1
            self.orien_const.absolute_z_axis_tolerance = 0.1
            self.orien_const.weight = 0.1
            
            self.right_gripper.calibrate()
        """
        # self.orien_const.link_name = "right_hand"
        # self.orien_const.header.frame_id = "base"

        # orientation_const = (-np.pi,0,0)
        # self.orien_const.orientation.x,self.orien_const.orientation.y,self.orien_const.orientation.z,self.orien_const.orientation.w    = quaternion_from_euler(np.pi,0,np.pi)
        # self.orien_const.absolute_x_axis_tolerance = 0.5
        # self.absolute_y_axis_tolerance = 0.5
        # self.orien_const.absolute_z_axis_tolerance = 0.5
        # self.orien_const.weight = 1
        

        self.right_gripper.calibrate()
        rospy.sleep(0.5)

        self.right_gripper.open()
        rospy.sleep(0.5)




        # ### Adding obstacles to improve the motion
        # # Adding an obstacle = table10
        # table_size = np.array([0.7,1,0.05])
        # table_name = "table"
        
        # table_pose = PoseStamped()
        # table_pose.header.frame_id = "base"
        # table_pose.pose.position.x = 0.7
        # table_pose.pose.position.y = 0
        # table_pose.pose.position.z = -0.25
        
        # table_pose.pose.orientation.x,table_pose.pose.orientation.y,table_pose.pose.orientation.z,table_pose.pose.orientation.w  = quaternion_from_euler(np.pi,0,np.pi)
        # self.planner.add_box_obstacle(table_size, table_name, table_pose)


        # Adding an obstacle = wall
        # wall_size = np.array([0.2,1,1.5])
        # wall_name = "wall"

        # wall_pose = PoseStamped()
        # wall_pose.header.frame_id = "base"
        # wall_pose.pose.position.x = -0.8
        # wall_pose.pose.position.y = 0.0
        # wall_pose.pose.position.z = 0.0

        # wall_pose.pose.orientation.x,wall_pose.pose.orientation.y,wall_pose.pose.orientation.z,wall_pose.pose.orientation.w = quaternion_from_euler(0,0,0)
        # self.planner.add_box_obstacle(wall_size, wall_name, wall_pose)


        self.planner._limb.move_to_neutral()


        # Moving to a more convenient position, for a better move next
        #self.going_to_standard_position()


        # Adding an obstacle = ceilling
        # ceil_size = np.array([0.7,1,0.05])
        # ceil_name = "ceil"

        # ceil_pose = PoseStamped()
        # ceil_pose.header.frame_id = "base"
        # ceil_pose.pose.position.x = 0.7
        # ceil_pose.pose.position.y = 0.0
        # ceil_pose.pose.position.z = 0.7

        # ceil_pose.pose.orientation.x,ceil_pose.pose.orientation.y,ceil_pose.pose.orientation.z,ceil_pose.pose.orientation.w = quaternion_from_euler(np.pi,0,np.pi)
        # self.planner.add_box_obstacle(ceil_size, ceil_name, ceil_pose)
    



                
        
    def analyzing_motion(self):

        x,y,z= 0.757, 0.069, 0.350

        """
        Going throught 2 steps and should see at least three components
        """
        plan = {'right_j0': -1.1822275390625,'right_j1': 0.501970703125,'right_j2': -1.3819697265625,
            'right_j3':-1.1907275390625,'right_j4': 1.0726474609375, 
                'right_j5': 0.0539248046875, 'right_j6': 1.7883544921875}

        velocities = {'right_j0': 0.2,'right_j1': 0,'right_j2': 0,'right_j3':0,'right_j4': 0, 'right_j5': 0, 'right_j6': 0}
        velocities_0 = {'right_j0': 0,'right_j1': 0,'right_j2': 0,'right_j3':0,'right_j4': 0, 'right_j5': 0, 'right_j6': 0}
        # init_position_for_scanning = PoseStamped()

        # init_position_for_scanning.header.frame_id = "base"

        # init_position_for_scanning.pose.position.x = x
        # init_position_for_scanning.pose.position.y = y
        # init_position_for_scanning.pose.position.z = z

        # init_position_for_scanning.pose.orientation.x,init_position_for_scanning.pose.orientation.y,init_position_for_scanning.pose.orientation.z,init_position_for_scanning.pose.orientation.w = 0.059 , 0.706 , -0.025 ,0.706
        # plan = self.planner.plan_to_pose(init_position_for_scanning,[])
        if not self.planner.execute_plan(plan): 
            raise Exception("Execution failed")

        rospy.sleep(1.0)
        i = 0
        while ((self.ar_track["1st_glass"] == 0) or (self.ar_track["2nd_glass"] == 0) or (self.ar_track["container"] == 0) ) and (i<7):
            # # Wait for the ar_pose_marker topic to become available
            
            rospy.loginfo("Waiting for ar_pose_marker topic...")
            rospy.wait_for_message('ar_pose_marker', AlvarMarkers)
            

            # Subscribe to the ar_pose_marker topic to get the image width and height
            rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.detect_ar)

            #rospy.sleep(1.0)
            self.planner._limb.set_joint_velocities(velocities)
            ## Motion to go to Z
            # init_position_for_scanning.pose.position.y += 0.02
            # plan = self.planner.plan_to_pose(init_position_for_scanning,[])
            """
            plan['right_j0'] += 0.1
            self.planner.execute_plan(plan)
            i += 1
            """
            
            print(self.ar_track)

        self.planner._limb.set_joint_velocities(velocities_0)
        print("Analyzing...")

        rospy.sleep(1.0)


        # while not rospy.is_shutdown():
        #     try :
        #         for i in range(3):
        #             goal = PoseStamped()
        #             goal.header.frame_id = "base"
                
        #             #x, y, and z position
        #             """
        #             We will need to tune it in order to be sure that the glass shiuld be easily poured after that
        #             """
        #             goal.pose.position.x = x
        #             goal.pose.position.y = y+0.2*i
        #             goal.pose.position.z = z
                
        #             #Orientation as a quaternion
        #             # goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w = quaternion_from_euler(-np.pi,-np.pi/2,0)    
        #             goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w = quaternion_from_euler(-np.pi,-np.pi/2,0)

                    
        #             plan = self.planner.plan_to_pose(goal, [self.orien_const])
        #             rospy.sleep(2.0)

                    
        #             if not self.planner.execute_plan(plan): 
        #                     raise Exception("Execution failed")
        #     except Exception as e:
        #         print(e)
        #         traceback.print_exc()
        #     else:
        #         break

        self.planner._limb.move_to_neutral()
        self.going_to_standard_position()

        


            






        return True


    def detect_ar(self,msg):
        for elt in msg.markers:
            # elt.pose
            pose_wrt_camera = PoseStamped()
            pose_wrt_camera.header.frame_id = "right_hand_camera" #Camera_id

            pose_wrt_camera.pose = elt.pose
            ### tf to the base:

            # tf_list = tf.TransformListener()
            # pose_wrt_base = tf_list.transformPose('base',pose_wrt_camera)

            # staticBroad.header.frame_id="base"
            # staticBroad.child_frame_id = elt.header.frame_id
            
            # 
            #  
            #  
            
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

        Going to the std posiiton (Phase 0)
        Can be used inside the other position /!\ with the gripper
        
        """
        """
        rospy.sleep(0.5)
        # plan = {'joint_j0':-8.23227276278149e-11,'joint_j1': -1.0,'joint_j2':  8.355809620608223e-11,
        #     'joint_j3': 1.0000000000197249, 'joint_j4': -1.5382754749897545e-09,
        #         'joint_j5': 1.5999999960915086, 'joint_j6': 1.5707963267900003}

        # if not self.planner.execute_plan(plan): 
        #     raise Exception("Execution failed")
        self.planner._limb.move_to_neutral()

        rospy.sleep(.5)
       
        
        
        """
        
        while not rospy.is_shutdown():
            try :
                x,y,z = 0.6,0.2,0.2
                
                goal = PoseStamped()
                goal.header.frame_id = "base"
            
                #x, y, and z position
                #We will need to tune it in order to be sure that the glass shiuld be easily poured after that
                
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.position.z = z
            
                #Orientation as a quaternion
                # goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w = quaternion_from_euler(-np.pi,-np.pi/2,0)    
                goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w = quaternion_from_euler(-np.pi,-np.pi/2,0)

                
                plan = self.planner.plan_to_pose(goal, [self.orien_const])

                
                if not self.planner.execute_plan(plan): 
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break
        
        return True
            
    def open(self):
        self.right_gripper.open()
        rospy.sleep(0.5)

    def close(self):
        self.right_gripper.close()
        rospy.sleep(0.5)
        
    def move_to_position(self, glass_position, x_bias=0.0, y_bias=0.0, z_bias=0.0):
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
        We do phase A
        
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
                #goal_1.pose.orientation.x,goal_1.pose.orientation.y,goal_1.pose.orientation.z,goal_1.pose.orientation.w = quaternion_from_euler(np.pi,0,np.pi)
                goal_1.pose.orientation.x,goal_1.pose.orientation.y,goal_1.pose.orientation.z,goal_1.pose.orientation.w = quaternion_from_euler(-np.pi,-np.pi/2,0)
                
                plan_1 = self.planner.plan_to_pose(goal_1, [self.orien_const])
                #input("Press <Enter> to move the right arm to the 1st glass: ")
                
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
            
            
        ### Going to standard position
        self.going_to_standard_position()
            
        
            
        return True
            
            
            
            
            
        
        
        
        
    def going_to_container_and_pouring(self, container_position, x_bias=0.0, y_bias=0.0, z_bias=0.0):
        """
        We do phase B
        
        """
        x,y,z = container_position.position.x,container_position.position.y,container_position.position.z
        x += x_bias
        y += y_bias
        z += z_bias
        #Container position
        # ### Going to the container
        # # step 1 : going above

        # while not rospy.is_shutdown():
        #     try :
                
        #         goal_2 = PoseStamped()
        #         goal_2.header.frame_id = "base"
            
        #         #x, y, and z position
        #         goal_2.pose.position.x = x
        #         goal_2.pose.position.y = y
        #         goal_2.pose.position.z = 0.166
            
        #         #Orientation as a quaternion
        #         goal_2.pose.orientation.x,goal_2.pose.orientation.y,goal_2.pose.orientation.z,goal_2.pose.orientation.w = quaternion_from_euler(np.pi,0,np.pi)
                
                
        #         plan_2 = self.planner.plan_to_pose(goal_2, [self.orien_const])
        #         input("Press <Enter> to move the right arm to the container: ")
                
        #         if not self.planner.execute_plan(plan_2): 
        #                 raise Exception("Execution failed")
        #     except Exception as e:
        #         print(e)
        #         traceback.print_exc()
        #     else:
        #         break


       # self.going_to_standard_position()
            
        ### Pouring
                
                # Acting on the joint directly 
            
        plan_45 = None 
             
                
            # " first Motion = going down"
            # " Then waiting"
            # " Then going up"
            
        # First move it a bit on the side 
        # And orientation to pour it
        while not rospy.is_shutdown():
            try :
                
                goal_3 = PoseStamped()
                goal_3.header.frame_id = "base"
            
                #x, y, and z position
                goal_3.pose.position.x = x 
                goal_3.pose.position.y = y
                goal_3.pose.position.z = z
            
                #Orientation as a quaternion
                # goal_3.pose.orientation.x,goal_3.pose.orientation.y,goal_3.pose.orientation.z,goal_3.pose.orientation.w = quaternion_from_euler(np.pi,0,np.pi)
                goal_3.pose.orientation.x,goal_3.pose.orientation.y,goal_3.pose.orientation.z,goal_3.pose.orientation.w = quaternion_from_euler(-np.pi,-np.pi/2,0)
                
                plan_3 = self.planner.plan_to_pose(goal_3, [self.orien_const])
                plan_45 = plan_3
                # input("Press <Enter> to move the right arm above the container ")
                
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
                

                # goal_4 = PoseStamped()
                # goal_4.header.frame_id = "base"
            
                # #x, y, and z position
                # goal_4.pose.position.x = x
                # goal_4.pose.position.y = y 
                # goal_4.pose.position.z = -0.1
            
                # #Orientation as a quaternion
                # # goal_4.pose.orientation.x,goal_4.pose.orientation.y,goal_4.pose.orientation.z,goal_4.pose.orientation.w = quaternion_from_euler(np.pi,0,np.pi)
                # goal_4.pose.orientation.x,goal_4.pose.orientation.y,goal_4.pose.orientation.z,goal_4.pose.orientation.w = quaternion_from_euler(-np.pi,-np.pi/2,0)
                
                
                # plan_4 = self.planner.plan_to_pose(goal_4, [self.orien_const])
                plan_4 = plan_45.copy()
                print(plan_4)
                plan_4['right_j6'] -= 1.75
                #input("Press <Enter> to move the right arm to pour the glass")

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
                
                # goal_5 = PoseStamped()
                # goal_5.header.frame_id = "base"
            
                # #x, y, and z position
                # goal_5.pose.position.x = x
                # goal_5.pose.position.y = y 
                # goal_5.pose.position.z = -0.1
            
                # #Orientation as a quaternion
                # # goal_5.pose.orientation.x,goal_5.pose.orientation.y,goal_5.pose.orientation.z,goal_5.pose.orientation.w = quaternion_from_euler(np.pi,0,np.pi)
                # goal_5.pose.orientation.x,goal_5.pose.orientation.y,goal_5.pose.orientation.z,goal_5.pose.orientation.w = quaternion_from_euler(-np.pi,-np.pi/2,0)

                
                
                # plan_5 = self.planner.plan_to_pose(goal_5, [self.orien_const])
                plan_5 = plan_45
                
                if not self.planner.execute_plan(plan_5): 
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break           
        
        rospy.sleep(0.5)

        ### Going to standard position
        #self.going_to_standard_position()
        
        return True
                        
            
        
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
                # goal_1.pose.orientation.x,goal_1.pose.orientation.y,goal_1.pose.orientation.z,goal_1.pose.orientation.w = quaternion_from_euler(np.pi,0,np.pi)
                goal_1.pose.orientation.x,goal_1.pose.orientation.y,goal_1.pose.orientation.z,goal_1.pose.orientation.w = quaternion_from_euler(-np.pi,-np.pi/2,0)
                # input("Press <Enter> to move the right arm to the initial glass position")

                
                
                plan_1 = self.planner.plan_to_pose(goal_1, [self.orien_const])
                
                if not self.planner.execute_plan(plan_1): 
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break
            
        ### Opening the gripper
        rospy.sleep(.5)

        print("Opening Gripper")
        self.right_gripper.open()
        rospy.sleep(.5)      
        
        ### Going to standard position
        self.going_to_standard_position()
        
        return True   
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            