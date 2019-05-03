#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on April 25 2019

@authors: Bharat C Renjith(bchenna@ncsu.edu),
		 Janani J Kalaivani(gjanaga@ncsu.edu)
		 Lokeshwar Deenadayalan(ldeenad@ncsu.edu),
		 Mohnish Ramani(mramani@ncsu.edu) 
"""

import rospy,sys
import time
import os
from vehicle import Vehicle
from utilities import Utils
from geometry_msgs.msg import PoseStamped
import cv2
import yolo
import v4l2_cam as cam
from wolfdrone.msg import waypoint
from wolfdrone.msg import human

#TODO: Change the relative path here
sys.path.append('/home/nvidia/catkin_ws/src/wolfdrone/scripts/mission/lib')
sys.path.append('/home/nvidia/catkin_ws/src/wolfdrone/scripts/mission/utils')
sys.path.append('/home/nvidia/catkin_ws/src/wolfdrone/scripts/mission')
sys.path.append('/home/nvidia/catkin_ws/src/wolfdrone/scripts/mission/lib/camera')


# Flag if set to False will bypass human detection
takePic = True

class Simple_Mission():
  def __init__(self):
    rospy.init_node("simple_mission",log_level=rospy.INFO)
    #Publisher for Waypoint message
    self.pub = rospy.Publisher("/wolfdrone/Waypoint",waypoint,queue_size=10)
    #Subscriber for humanDetector message
    rospy.Subscriber("/wolfdrone/humanDetector",human,self.callback,queue_size =10)
    
    #Flag gets updated when human_detector.py sends a message
    self.update_flag = False 
    
    #Flag gets updated when human_detector.py finds Homo Sapien ! 
    self.human_flag = False
    
    #His/Her Coordinates are updated in this attribute
    self.human_x = 0
    self.human_y = 0

    self.msg = waypoint()
    self.drone = Vehicle()
    
    self.drone.home_lat = rospy.get_param("~home_lat")
    self.drone.home_lon = rospy.get_param("~home_lon")
    self.drone.drop_lat = rospy.get_param("~drop_lat")
    self.drone.drop_lon = rospy.get_param("~drop_lon")
    self.drone.land_lat = rospy.get_param("~land_lat")
    self.drone.land_lon = rospy.get_param("~land_lon")

  def callback(self,data):
    print("Inside Waypoint Callback")
    self.update_flag = data.lookforCameraResponseFlag
    self.human_flag = data.humanFlag
    self.human_x = data.x
    self.human_y = data.y

  def startup(self):
    #Load the mission waypoints
    self.wp_list = Utils.single_wp()

  def mission(self):

    ### PX4 rejects the offboard control mode without few setpoint commands already.
    ### So adding arbitrary number of waypoints before making offboard
    rate = rospy.Rate(10)
    t = self.wp_list[-1]
    pose = PoseStamped()
    pose.pose.position.x = t[0]
    pose.pose.position.y = t[1]
    pose.pose.position.z = t[2]

    for i in range(10):
      self.drone.setpoint_pose(pose)
      rate.sleep()
    print("Sent Arbitrary Number of waypoints")
    self.drone.command_mode('OFFBOARD')
    self.drone.arm()
    
    print("List of waypoints")
    print(self.wp_list)
 
    
    ### Waypoint Navigation code
    counter = 0
    mission_completed = False
    rate = rospy.Rate(100)
    while mission_completed == False:
      rate.sleep()
      # set drone pose to way point if drone hasn't reached there yet
      if abs(Utils.compute_distance(pose,self.drone.pose)) > 0.5:
        self.drone.setpoint_pose(pose)
                
      else:
        print("Waypoint is reached")
        #Publish the Waypoint Reached Flag for camera to take picture
        self.msg.waypointReachedFlag = True
        self.pub.publish(self.msg)
        
        # just a flag to disable human detection and change back to traversing set way points
        if takePic:
            # While human detection node has not published any new info set pose as current pose
            start_time = time.time()
            while(self.update_flag == False):
                self.drone.setpoint_pose(pose)
                # If human detection node has not published any info for 3 seconds continue scanning
                if(time.time() - start_time > 3.0):
                    print("Human detector did not publish any new messages")
                    self.human_flag = False
                    break
            self.msg.waypointReachedFlag = False
            self.pub.publish(self.msg)
            # Human Detection node has published message, Now check if human detected
            if self.human_flag == True:
                print("Human detected in the area! Drone to the Rescue!!!")
                
                if pose.pose.position.z <  5:
                    print("Reached Human. Enter HOLD mode")
                    mission_completed = True
                    break
                else:
                    pose.pose.position.z -= 2
                    
                # Human Coordinates are relative to the drone's position
                pose.pose.position.x = self.human_x + pose.pose.position.x
                pose.pose.position.y = self.human_y + pose.pose.position.y
                
            else:
                # If all the waypoints have been traversed update mission completed flag
                print("Human not detected in the area. Moving on to scanning...")
                if len(self.wp_list) == 0:
                    mission_completed = True
                    break
                t = self.wp_list.pop()
                pose.pose.position.x = t[0]
                pose.pose.position.y = t[1]
                pose.pose.position.z = t[2]
            
        else: #/* Else part of if take pic */
            if len(self.wp_list) == 0:
                mission_completed = True
                break
            t = self.wp_list.pop()
            # Normal wayopint traversal
            pose.pose.position.x = t[0]
            pose.pose.position.y = t[1]
            pose.pose.position.z = t[2]
        # Disable way point reached flag 
        self.msg.waypointReachedFlag = False
        self.pub.publish(self.msg)
        
        print(t)

#   Once Reached near Human, HOLD at the current location
    start_time = time.time()
    while time.time() - start_time < 3:
        continue
    print("Position held for 3 seconds.Land Initiated")
    land_pose = PoseStamped()
    #land_local = Utils.convert_global2local(self.drone.home_lat,self.drone.home_lon,self.drone.land_lat,self.drone.land_lon)
    land_pose.pose.position.x = 0#land_local[0]
    land_pose.pose.position.y = 0#land_local[1]
    land_pose.pose.position.z = 2


    for i in range(10):
      self.drone.setpoint_pose(land_pose)
      rate.sleep()
    print("Sent Arbitrary Number of waypoints")
    self.drone.command_mode('OFFBOARD')
    # Move towards land position
    while abs(Utils.compute_distance(land_pose,self.drone.pose)) > 0.5:
        self.drone.setpoint_pose(land_pose)
        rate.sleep()
    self.drone.land(self.drone.land_lat,self.drone.land_lon)
    print("Mission Complete")

  def cleanup(self):
    pass


  def run(self):

    self.startup()
    self.mission()
    self.cleanup()


if __name__== '__main__':
    try:
        mission = Simple_Mission()
        print("Initialized Simple Mission,Starting Mission")
        #Time for YOLO to load its weights
        time.sleep(20)
        mission.run()
    except Exception as e:
        print(e)
