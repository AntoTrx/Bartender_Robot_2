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
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
from controller import Controller

import math_functions
from tf.transformations import quaternion_from_euler


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

        self.limb = Limb('right')


        Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        self.controller = Controller(Kp, Ki, Kd, Kw,self.limb)
        
        
        
        
        
            
        
        

        
    
     
    def initialize(self):
        """
        A function to calibrate the gripper and put the robot in the standard position
        
        """
        # Doing the left to right position in order to get all the positions
        
        
        
        
        
        
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
        self.orien_const.link_name = "right_hand"
        self.orien_const.header.frame_id = "base"

        orientation_const = (-np.pi,0,0)
        self.orien_const.orientation.x,self.orien_const.orientation.y,self.orien_const.orientation.z,self.orien_const.orientation.w    = quaternion_from_euler(np.pi,0,np.pi)
        self.orien_const.absolute_x_axis_tolerance = 0.5
        self.absolute_y_axis_tolerance = 0.5
        self.orien_const.absolute_z_axis_tolerance = 0.5
        self.orien_const.weight = 1
            
        self.right_gripper.calibrate()
        # rospy.sleep(2.0)


        ### Adding obstacles to improve the motion
        # Adding an obstacle = table10
        table_size = np.array([0.7,1,0.05])
        table_name = "table"
        
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base"
        table_pose.pose.position.x = 0.7
        table_pose.pose.position.y = 0
        table_pose.pose.position.z = -0.25
        
        table_pose.pose.orientation.x,table_pose.pose.orientation.y,table_pose.pose.orientation.z,table_pose.pose.orientation.w  = quaternion_from_euler(np.pi,0,np.pi)
        self.planner.add_box_obstacle(table_size, table_name, table_pose)


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


        # Moving to a more convenient position, for a better move next
        self.going_to_standard_position()


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
    

        print("Add obstacle")


                
        
        
    
    def going_to_standard_position(self):
        """

        Going to the std posiiton (Phase 0)
        Can be used inside the other position /!\ with the gripper
        
        """
        
        while not rospy.is_shutdown():
            try :
                x,y,z = 0.8, 0.158, 0.01
                
                goal = PoseStamped()
                goal.header.frame_id = "base"
            
                #x, y, and z position
                """
                We will need to tune it in order to be sure that the glass shiuld be easily poured after that
                """
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.position.z = z
            
                #Orientation as a quaternion
                goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w = quaternion_from_euler(3.137, -0.027, -2.969)
                
                
                plan = self.planner.plan_to_pose(goal, [self.orien_const])
                input("Press <Enter> to move the right arm to standard position: ")
                
                if not self.controller.execute_plan(plan[1]): 
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break
        
        return True
            
        
        
        
    
    def going_and_take_glass(self,glass_position):
        """
        We do phase A
        
        """
        
        
        # Going to the glass,
        # step 1 : going above:
        while not rospy.is_shutdown():
            try :
                x,y,z = glass_position
                
                goal_0 = PoseStamped()
                goal_0.header.frame_id = "base"
            
                #x, y, and z position
                goal_0.pose.position.x = x
                goal_0.pose.position.y = y
                goal_0.pose.position.z = 0.074
            
                #Orientation as a quaternion
                goal_0.pose.orientation.x,goal_0.pose.orientation.y,goal_0.pose.orientation.z,goal_0.pose.orientation.w = quaternion_from_euler(np.pi,0,np.pi)
                
                
                plan_0 = self.planner.plan_to_pose(goal_0, [self.orien_const])
                input("Press <Enter> to move the right arm above to the 1st glass: ")
                
                if not self.controller.execute_plan(plan_0[1]): 
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break


        # Step 2 Going to the glass
        while not rospy.is_shutdown():
            try :
                x,y,z = glass_position
                
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"
            
                #x, y, and z position
                goal_1.pose.position.x = x
                goal_1.pose.position.y = y
                goal_1.pose.position.z = z
            
                #Orientation as a quaternion
                goal_1.pose.orientation.x,goal_1.pose.orientation.y,goal_1.pose.orientation.z,goal_1.pose.orientation.w = quaternion_from_euler(np.pi,0,np.pi)
                
                
                plan_1 = self.planner.plan_to_pose(goal_1, [self.orien_const])
                input("Press <Enter> to move the right arm to the 1st glass: ")
                
                if not self.controller.execute_plan(plan_1[1]): 
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break
        
        
        # Closing the gripper
        print("Closing Gripper")
        self.right_gripper.close()
        rospy.sleep(1.0)
            
            
        ### Going to standard position
        self.going_to_standard_position()
            
        
            
        return True
            
            
            
            
            
        
        
        
        
    def going_to_container_and_pouring(self, container_position):
        """
        We do phase B
        
        """
        ### Going to the container
        # step 1 : going above
        while not rospy.is_shutdown():
            try :
                x,y,z = container_position #Container position
                
                goal_2 = PoseStamped()
                goal_2.header.frame_id = "base"
            
                #x, y, and z position
                goal_2.pose.position.x = x
                goal_2.pose.position.y = y
                goal_2.pose.position.z = 0.074
            
                #Orientation as a quaternion
                goal_2.pose.orientation.x,goal_2.pose.orientation.y,goal_2.pose.orientation.z,goal_2.pose.orientation.w = quaternion_from_euler(np.pi,0,np.pi)
                
                
                plan_2 = self.planner.plan_to_pose(goal_2, [self.orien_const])
                input("Press <Enter> to move the right arm to the container: ")
                
                if not self.controller.execute_plan(plan_2[1]): 
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break
            
        ### Pouring
                """
                Acting on the joint directly 
            
                
                """
                
                " first Motion = going down"
                " Then waiting"
                " Then going up"
                
            # First move it a bit on the side 
            # And orientation to pour it
            while not rospy.is_shutdown():
                try :
                    
                    goal_3 = PoseStamped()
                    goal_3.header.frame_id = "base"
                
                    #x, y, and z position
                    goal_3.pose.position.x = goal_2.pose.position.x 
                    goal_3.pose.position.y = goal_2.pose.position.y +0.1
                    goal_3.pose.position.z = goal_2.pose.position.z 
                
                    #Orientation as a quaternion
                    goal_3.pose.orientation.x,goal_3.pose.orientation.y,goal_3.pose.orientation.z,goal_3.pose.orientation.w = quaternion_from_euler(np.pi,0,np.pi/2)
                
                    
                    plan_3 = self.planner.plan_to_pose(goal_3, [self.orien_const])
                    input("Press <Enter> to move the right arm to pour the glass next ")
                    
                    if not self.controller.execute_plan(plan_3[1]): 
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
                    
                    goal_4 = PoseStamped()
                    goal_4.header.frame_id = "base"
                
                    #x, y, and z position
                    goal_4.pose.position.x = goal_3.pose.position.x 
                    goal_4.pose.position.y = goal_3.pose.position.y
                    goal_4.pose.position.z = goal_3.pose.position.z 
                
                    #Orientation as a quaternion
                    goal_4.pose.orientation.x,goal_4.pose.orientation.y,goal_4.pose.orientation.z,goal_4.pose.orientation.w = quaternion_from_euler(np.pi,np,np.pi/2,np.pi/2)
                    
                    
                    plan_4 = self.planner.plan_to_pose(goal_4, [self.orien_const])
                    input("Press <Enter> to move the right arm to pour the glass")

                    if not self.controller.execute_plan(plan_4[1]): 
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
                    
                    goal_5 = PoseStamped()
                    goal_5.header.frame_id = "base"
                
                    #x, y, and z position
                    goal_5.pose.position.x = goal_3.pose.position.x 
                    goal_5.pose.position.y = goal_3.pose.position.y 
                    goal_5.pose.position.z = goal_3.pose.position.z 
                
                    #Orientation as a quaternion
                    goal_5.pose.orientation.x,goal_5.pose.orientation.y,goal_5.pose.orientation.z,goal_5.pose.orientation.w = quaternion_from_euler(np.pi,0,np.pi)
                    
                    
                    
                    plan_5 = self.planner.plan_to_pose(goal_5, [self.orien_const])
                    
                    if not self.controller.execute_plan(plan_5[1]): 
                            raise Exception("Execution failed")
                except Exception as e:
                    print(e)
                    traceback.print_exc()
                else:
                    break           
        
        ### Going to standard position
        self.going_to_standard_position()
        
        return True
                        
            
        
    def putting_glass_back(self,glass_position):
        """
        Putting again the glass at the original position
        """
        ### Going to the initial location
        while not rospy.is_shutdown():
            try :
                x,y,z = glass_position # Going to the initial configuration
                
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"
            
                #x, y, and z position
                goal_1.pose.position.x = x
                goal_1.pose.position.y = y
                goal_1.pose.position.z = z
            
                #Orientation as a quaternion
                goal_1.pose.orientation.x,goal_1.pose.orientation.y,goal_1.pose.orientation.z,goal_1.pose.orientation.w = quaternion_from_euler(np.pi,0,np.pi)
                input("Press <Enter> to move the right arm to the initial glass position")

                
                
                plan_1 = self.planner.plan_to_pose(goal_1, [self.orien_const])
                
                if not self.controller.execute_plan(plan_1[1]): 
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break
            
        ### Opening the gripper
        print("Opening Gripper")
        self.right_gripper.open()
        rospy.sleep(1.0)      
        
        ### Going to standard position
        self.going_to_standard_position()
        
        return True   
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            