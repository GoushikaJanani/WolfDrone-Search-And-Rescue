#!/usr/bin/env python
"""
Created on Sat Mar 24 23:34:52 2018

@author: vijayaganesh
"""

import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped,Vector3Stamped

from mavros_msgs.msg import  ParamValue

from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, ParamSet

from position import Positions

class Vehicle(object,Positions):
  """
  Class for arming and Prearm flight Checks
  
  TODO: Add Codebase to perform prearm checks
  """
  
  def __init__(self):
    Positions.__init__(self)
    self.panic = False #Panic variable as failsafe
    
#   The following parameters will be sent using rosparam from roslaunch
    self.home_lat = None
    self.home_lon = None
    self.drop_lat = None
    self.drop_lon = None
    self.land_lat = None
    self.land_lon = None
    
    self.pass_prearm = True #TODO Alter this parameter after adding the codebase for prearm check
    self.setpoint_publisher = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    self.setvel_publisher = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size = 10)
    self.setaccel_publisher = rospy.Publisher("/mavros/setpoint_accel/accel",Vector3Stamped,queue_size=10)
    
  def command_offboard(self):
    arbitrary_pose= PoseStamped()
    arbitrary_pose.pose = self.pose.pose
    temp_rate = rospy.Rate(10)
    for i in range(10):
      self.setpoint_pose(arbitrary_pose)
      temp_rate.sleep()
    self.command_mode()
  
  def command_mode(self,mode='OFFBOARD'):
    rospy.wait_for_service('/mavros/set_mode')
    command_service = rospy.ServiceProxy('/mavros/set_mode',SetMode)
    num = 0 if mode == 'OFFBOARD' else 1
    command_service(num,mode)
    print ("Mode Changed :"+mode)
    
  def set_xy_speed(self,speed):
    rospy.wait_for_service('/mavros/param/set')
    speed_service = rospy.ServiceProxy('/mavros/param/set',ParamSet)
    speed_service('MPC_XY_VEL_MAX',ParamValue(0,speed))
    
  def land(self,land_lat,land_lon):
    rospy.wait_for_service('/mavros/cmd/land')
    land_service = rospy.ServiceProxy('/mavros/cmd/land',CommandTOL)
    land_service(0,0,land_lat,land_lon,0)
    print("Land Initiated")
    rate = rospy.Rate(10)
    for i in range(5):
      rate.sleep()
    prev_vel = self.velocity.twist.linear.z
    curr_vel = self.velocity.twist.linear.z
    print((prev_vel,curr_vel))
    while not (abs(prev_vel) < 0.05 and abs(curr_vel) < 0.05):
      print("Waiting to Land")
      print((prev_vel,curr_vel))
      curr_vel = self.velocity.twist.linear.z
      prev_vel = curr_vel
      rate.sleep()
    
  def setpoint_pose(self,set_pose):    
    self.setpoint_publisher.publish(set_pose)
  
  def setpoint_vel(self,set_vel):
    self.setvel_publisher.publish(set_vel)
    
  def setpoint_accel(self,set_accel):
    self.setaccel_publisher.publish(set_accel)
  
  def arm(self):
    try:
      assert self.pass_prearm
    except AssertionError:
      print("Pre-Arm Failed! Going to Panic Mode")
      self.panic = True
      return
    self.command_arm(True)
    print("Vehicle Armed")
    
      
  def disarm(self):
    """
      TODO: Add code to check whether the drone has landed or in safe altitude before disarming
    """    
    self.command_arm(False)
    print("Vehicle Disarmed")
  
  def command_arm(self,arm):
    rospy.wait_for_service('/mavros/cmd/arming')
    arm_service = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
    arm_service(arm)
