#!/usr/bin/env python

from PyQt5 import QtCore, QtGui, QtWidgets
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pandas as pd 
from random import seed
from random import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.utils\
    import ControllerLister, ControllerManagerLister
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def neuralNetwork():
    pub = rospy.Publisher('mobile_base_controller/cmd_vel', Twist)
    rospy.init_node('test', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    move_cmd = Twist()
    move_cmd.linear.x = 1.0
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(move_cmd)
        rate.sleep


class neuralNetwork(object):
    def __init__(self):
        rospy.init_node("AI_agent")
        self.image_subI = rospy.Subscriber("/xtion/rgb/image_color",Image,self.image_callback)
        self.image_subD = rospy.Subscriber("/xtion/depth_registered/image_raw",Image,self.imageD_callback)
        self.arm_listener = rospy.Subscriber("/arm_controller/state",JointTrajectoryControllerState,self.arm_callback)
        self.gripper_listener = rospy.Subscriber("/gripper_controller/state",JointTrajectoryControllerState,self.gripper_callback)
        self.torso_listener = rospy.Subscriber('/torso_controller/state',JointTrajectoryControllerState,self.torso_callback)
        self.torsohead_listener = rospy.Subscriber('/head_controller/state',JointTrajectoryControllerState,self.torsohead_callback)
        self.subscriberRedObject = rospy.Subscriber('mobile_base_controller/cmd_vel', Twist, self.movebase_callback)
        self.bridge = CvBridge()
        self.pub_mov = rospy.Publisher('mobile_base_controller/cmd_vel', Twist)
        self.torso_cmd = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1)
        self.head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
        self.arm_cmd = rospy.Publisher('/arm_controller/command',JointTrajectory,  queue_size=1)
        self.gripper_cmd = rospy.Publisher('gripper_controller/command',JointTrajectory, queue_size=1)
        self.movebase_linear_x = 0
        self.movebase_linear_y = 0
        self.movebase_linear_z = 0 
        self.movebase_angular_x = 0
        self.movebase_angular_y = 0
        self.movebase_angular_z = 0



    def image_callback(self,ImageRGB):
        cv_imageRGB = self.bridge.imgmsg_to_cv2(ImageRGB, "bgr8")
        down_width = 128
        down_height = 96
        down_points = (down_width, down_height)
        self.cv_imageRGB_resized = cv2.resize(cv_imageRGB,down_points)
    # Remove this part for test without depth
    def imageD_callback(self,ImageD):
        cv_imageD = self.bridge.imgmsg_to_cv2(ImageD, "8UC1")
        down_width = 128
        down_height = 96
        down_points = (down_width, down_height)
        self.cv_imageD_resized = cv2.resize(cv_imageD,down_points,interpolation = cv2.INTER_CUBIC)
    
    def movebase_callback(self,movebase_msg):
        self.movebase_linear_x = movebase_msg.linear.x
        self.movebase_linear_y = movebase_msg.linear.y
        self.movebase_linear_z = movebase_msg.linear.z
        self.movebase_angular_x = movebase_msg.angular.x
        self.movebase_angular_y = movebase_msg.angular.y
        self.movebase_angular_z = movebase_msg.angular.z

    def arm_callback(self,arm_information):
        #rospy.loginfo(arm_information.actual.positions[0])
        self.arm_joint1 = arm_information.actual.positions[0]
        self.arm_joint2 = arm_information.actual.positions[1]
        self.arm_joint3 = arm_information.actual.positions[2]
        self.arm_joint4 = arm_information.actual.positions[3]
        self.arm_joint5 = arm_information.actual.positions[4]
        self.arm_joint6 = arm_information.actual.positions[5]
    
    def gripper_callback(self,gripper_information):
        self.gripper_joint1 = gripper_information.actual.positions[0]
        self.gripper_joint2 = gripper_information.actual.positions[1]
    def torso_callback(self,torso_information):
        self.torso_joint = torso_information.actual.positions[0]
    def torsohead_callback(self,torsohead_information):
        self.torsohead_joint1 = torsohead_information.actual.positions[0]
        self.torsohead_joint2 = torsohead_information.actual.positions[1]

    def armMototion(self):
        rospy.loginfo("hand Adjustment")
        jt = JointTrajectory()
        jt.joint_names = ['arm_1_joint','arm_2_joint','arm_3_joint','arm_4_joint','arm_5_joint','arm_6_joint','arm_7_joint' ]
        jtp = JointTrajectoryPoint()
        jtp.positions = [self.arm_joint1_precited  , self.arm_joint2_precited , self.arm_joint3_precited 
        ,self.arm_joint4_precited  ,  self.arm_joint5_precited , self.arm_joint6_precited ]
        jtp.time_from_start = rospy.Duration(0.01)
        jt.points.append(jtp)
        self.arm_cmd.publish(jt)
    
    def action_head(self):
        rospy.loginfo("head action")
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [self.torsohead_joint1_predicted, self.torsohead_joint2_predicted]
        jtp.time_from_start = rospy.Duration(0.01)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)
    
    def action_torso(self):
        rospy.loginfo("torso action")
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [self.torso_joint_predicted]
        jtp.time_from_start = rospy.Duration(0.01)
        jt.points.append(jtp)
        self.torso_cmd.publish(jt)
    def action_gripper(self):
        rospy.loginfo("torso gripper action")
        jt = JointTrajectory()
        jt.joint_names = [ 'gripper_right_finger_joint', 'gripper_left_finger_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [self.gripper_joint1_predicted, self.gripper_joint1_predicted]
        jtp.time_from_start = rospy.Duration(0.01)
        self.gripper_cmd.publish(jt)
    
    def move_base(self):
        move_cmd = Twist()
        move_cmd.linear.x = self.movebase_linear_x_predicted
        move_cmd.linear.y = self.movebase_linear_y_predicted
        move_cmd.linear.z = self.movebase_linear_z_predicted
        move_cmd.angular.x = self.movebase_angular_x_predicted
        move_cmd.angular.y = self.movebase_angular_y_predicted
        move_cmd.angular.z = self.movebase_angular_z_predicted
        self.pub_mov.publish(move_cmd)
        rospy.sleep(0.01)
        

    def run(self):
        rospy.loginfo("action is starting in 2 seconds")
        rospy.sleep(2.0)
        while not rospy.is_shutdown():
            ## Update predicted value via model:
            rospy.loginfo("running")
            self.arm_joint1_precited = self.arm_joint1
            self.arm_joint2_precited = self.arm_joint2
            self.arm_joint3_precited = self.arm_joint3
            self.arm_joint4_precited = self.arm_joint4
            self.arm_joint5_precited = self.arm_joint5
            self.arm_joint6_precited = self.arm_joint6
            self.armMototion()
            self.torsohead_joint1_predicted = self.torsohead_joint1
            self.torsohead_joint2_predicted = self.torsohead_joint2
            self.action_head()
            self.torso_joint_predicted = self.torso_joint
            self.gripper_joint1_predicted = self.gripper_joint1
            self.gripper_joint2_predicted = self.gripper_joint2
            self.action_gripper()
            self.movebase_linear_x_predicted = self.movebase_linear_x
            self.movebase_linear_y_predicted = self.movebase_linear_y
            self.movebase_linear_z_predicted = self.movebase_linear_z
            self.movebase_angular_x_predicted = self.movebase_angular_x
            self.movebase_angular_y_predicted = self.movebase_angular_y
            self.movebase_angular_z_predicted = self.movebase_angular_z
            self.move_base()
            rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        agent = neuralNetwork()
        agent.run()

    except rospy.ROSInterruptException:
        pass