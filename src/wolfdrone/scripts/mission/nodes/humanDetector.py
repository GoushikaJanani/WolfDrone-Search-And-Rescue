#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on Sun 14 April, 2019

@author: Janani J Kalaivani(gjanaga@ncsu.edu),
         Bharat C Renjith(bchenna@ncsu.edu),
         Lokeshwar Deenadayalan(ldeenad@ncsu.edu),
         Mohnish Ramani(mramani@ncsu.edu)
"""

import rospy,sys,os
from geometry_msgs.msg import Point
from wolfdrone.msg import waypoint
from wolfdrone.msg import human
import time
import cv2
import yolo
import v4l2_cam as cam

#TODO: Change relative path here
sys.path.append('/home/nvidia/catkin_ws/src/wolfdrone/scripts/mission/lib')
sys.path.append('/home/nvidia/catkin_ws/src/wolfdrone/scripts/mission/drivers')
sys.path.append('/home/nvidia/catkin_ws/src/wolfdrone/scripts/mission/nodes')
sys.path.append('/home/nvidia/catkin_ws/src/wolfdrone/scripts/mission/controller')
sys.path.append('/home/nvidia/catkin_ws/src/wolfdrone/scripts/mission/utils')
sys.path.append('/home/nvidia/catkin_ws/src/wolfdrone/scripts/mission')
sys.path.append('/home/nvidia/catkin_ws/src/wolfdrone/scripts/mission/lib/camera')

# Enable FLAG to save the images that are captured
SAVE_IMAGE = True

counter = 0 # Just a counter for saving images
class humanDetector():
    def __init__(self):
        rospy.init_node('humanDetector', anonymous=True)
        self.pub = rospy.Publisher('/wolfdrone/humanDetector',human , queue_size=10)
        self.wp_flag = False
        rospy.Subscriber("/wolfdrone/Waypoint", waypoint, self.callback, queue_size=10)
        
        self.cam = cam.Camera()
        #Initialize Camera parameters. Calculated through camera calibration
        self.f_x = 1136.5904
        self.f_y = 1140.1123
        self.c_x = 970.8983
        self.c_y = 547.9886
        
        # initialize YOLO weights
        self.net,self.meta = yolo.init_yolo()
        
        # Message container
        self.msg = human()
        
        
    def callback(self,data):
        print("Inside human Detector Callback")
        self.wp_flag = data.waypointReachedFlag

    def findCoordinates(self,center_x,center_y):
        #Drone flies at an altitude of 10 m 
        z = 10 
        # Map image coordinates to World coordinates 
        human_x = z*(center_x - self.c_x)/self.f_x
        human_y = z*(center_y - self.c_y)/self.f_y
        return (human_x,human_y)
        
    def run(self):
        global counter
        while not rospy.is_shutdown():
            if self.wp_flag == True:
                self.msg.lookforCameraResponseFlag = True
                
                # Read one dummy frame
                self.cam.read_frame()
                #Read a full image and split frames
                split_frames,full_image = self.cam.read_frame_split()
                if SAVE_IMAGE:
                    # Save Full image
                    print("Saving Image.............")
                    filename = "/home/nvidia/camera/Test_" + str(counter) + ".jpg"
                    cv2.imwrite(filename,full_image)
                # Run YOLO on all the 5 split frames
                bbox = yolo.detect_from_frames(self.net,self.meta,split_frames)
                
                print("YOLO Testing Completed")
                print(bbox)
                if len(bbox) == 0:
                    self.msg.humanFlag = False
                    self.msg.x = 0
                    self.msg.y = 0 
                else:
                    (x1,y1,x2,y2) = bbox[0]
                    if SAVE_IMAGE:
                    # Save Full image
                    #TODO: Change relative path here
                        filename = "/home/nvidia/camera/Test_detection" + str(counter) + ".jpg"
                        cv2.putText(full_image,"person" ,(int(x1-x2/2) -2,int(y1-y2/2)-2),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
                        cv2.rectangle(full_image,(int(x1-x2/2),int(y1-y2/2)),(int(x1+x2/2),int(y1+y2/2)),(0,0,1),2)
                        cv2.imwrite(filename,full_image)
                    
                    human_coordinates = self.findCoordinates(x1,y1)
                    self.msg.humanFlag = True
                    self.msg.x = human_coordinates[0]
                    self.msg.y = human_coordinates[1]                    
                
                self.pub.publish(self.msg)
                # wait for 2 seconds to make sure message is subscribed
                rospy.sleep(2)
                print("Resetting the published data")
                self.msg.humanFlag = False
                self.msg.lookforCameraResponseFlag = False
                self.msg.x = 0
                self.msg.y = 0
                self.pub.publish(self.msg)
                if SAVE_IMAGE:
                    counter += 1

if __name__ == '__main__':
    try:
        obj = humanDetector()
        print("Initialized. Started Running.......")
        obj.run()
        print("Closing the video........")
        obj.cam.video.close()
        			
    except Exception as e:
        print(e)
