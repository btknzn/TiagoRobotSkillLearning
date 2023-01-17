#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pandas as pd 
from random import seed
from random import random
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.utils\
import ControllerLister, ControllerManagerLister
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from models import *
import ros_numpy
import torch


from PyQt5 import QtCore, QtGui, QtWidgets
import rospy
from batu_training.msg import objectLocation
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped
from std_msgs.msg import Header

from geometry_msgs.msg import PointStamped
from scipy.spatial.transform import Rotation as R
import numpy as np
import rospy
import time
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
import numpy as np
from std_srvs.srv import Empty
import cv2
from cv_bridge import CvBridge
from moveit_msgs.msg import MoveItErrorCodes
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
import numpy as np
from std_srvs.srv import Empty
import cv2
from cv_bridge import CvBridge
import torchvision.transforms.functional as TF

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
        #self.bridge = CvBridge()
        self.pub_mov = rospy.Publisher('mobile_base_controller/cmd_vel', Twist)
        self.torso_cmd = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1)
        self.head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
        self.arm_cmd = rospy.Publisher('/arm_controller/command',JointTrajectory,  queue_size=1)
        self.gripper_cmd = rospy.Publisher('gripper_controller/command',JointTrajectory, queue_size=1)
        self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
        self.image_callback_runned = True
        self.imageD_callback_runned = True
        self.move_base_callback_runned = True
        self.arm_callback_runned = True
        self.gripper_callback_runned = True
        self.torso_callback_runned = True
        self.torsohead_callback_runned = True




    def image_callback(self,ImageRGB):
        #cv_imageRGB = self.bridge.imgmsg_to_cv2(ImageRGB, "bgr8")
        cv_imageRGB = ros_numpy.numpify(ImageRGB)
        down_width = 128
        down_height = 96
        down_points = (down_width, down_height)
        self.cv_imageRGB_resized = cv2.resize(cv_imageRGB,down_points)
        self.image_callback_runned = False
    # Remove this part for test without depth

    def imageD_callback(self,ImageD):
        
        # cv_imageD = self.bridge.imgmsg_to_cv2(ImageD, "8UC1")
        cv_imageD = ros_numpy.numpify(ImageD)
        mask = np.isnan(cv_imageD )
        cv_imageD [mask] = 0
        down_width = 128
        down_height = 96
        down_points = (down_width, down_height)
        self.cv_imageD_resized = cv2.resize(cv_imageD ,down_points,interpolation = cv2.INTER_CUBIC)
        #rospy.loginfo(self.cv_imageD_resized)
        self.imageD_callback_runned = False

    


    def arm_callback(self,arm_information):
        #rospy.loginfo(arm_information.actual.positions[0])
        self.arm_joint1 = arm_information.actual.positions[0]
        self.arm_joint2 = arm_information.actual.positions[1]
        self.arm_joint3 = arm_information.actual.positions[2]
        self.arm_joint4 = arm_information.actual.positions[3]
        self.arm_joint5 = arm_information.actual.positions[4]
        self.arm_joint6 = arm_information.actual.positions[5]
        self.arm_joint7 = arm_information.actual.positions[6]
        self.arm_callback_runned = False
    
    def gripper_callback(self,gripper_information):
        self.gripper_joint1 = gripper_information.actual.positions[0]
        self.gripper_joint2 = gripper_information.actual.positions[1]
        self.gripper_callback_runned = False
    def torso_callback(self,torso_information):
        self.torso_joint = torso_information.actual.positions[0]
        self.torso_callback_runned = False
    def torsohead_callback(self,torsohead_information):
        self.torsohead_joint1 = torsohead_information.actual.positions[0]
        self.torsohead_joint2 = torsohead_information.actual.positions[1]
        self.torsohead_callback_runned = False

    def armMototion(self):
        rospy.loginfo("hand Adjustment")
        jt = JointTrajectory()
        jt.joint_names = ['arm_1_joint','arm_2_joint','arm_3_joint','arm_4_joint','arm_5_joint','arm_6_joint','arm_7_joint' ]
        jtp = JointTrajectoryPoint()
        jtp.positions = [self.arm_joint1_precited  , self.arm_joint2_precited , self.arm_joint3_precited 
        ,self.arm_joint4_precited  ,  self.arm_joint5_precited , self.arm_joint6_precited, self.arm_joint7_precited ]
        jtp.time_from_start = rospy.Duration(0.01)
        jt.points.append(jtp)
        self.arm_cmd.publish(jt)
    
    def action_head(self):
        rospy.loginfo("head action")
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [self.torsohead_joint1_predicted, self.torsohead_joint2_predicted]
        jtp.time_from_start = rospy.Duration(0.1)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)
    
    def action_torso(self):
        rospy.loginfo("torso action")
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [self.torso_joint_predicted]
        jtp.time_from_start = rospy.Duration(0.1)
        jt.points.append(jtp)
        self.torso_cmd.publish(jt)
    def action_gripper(self):
        rospy.loginfo("torso gripper action")
        jt = JointTrajectory()
        jt.joint_names = [ 'gripper_right_finger_joint', 'gripper_left_finger_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [self.gripper_joint1_predicted, self.gripper_joint1_predicted]
        jtp.time_from_start = rospy.Duration(0.1)
        self.gripper_cmd.publish(jt)

    

    def prepare_robot(self):
        rospy.loginfo("Unfold arm safely")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'pregrasp'
        pmg.skip_planning = False
        self.play_m_as.send_goal_and_wait(pmg)
        rospy.loginfo("Done.")
        rospy.loginfo("Robot prepared.")

    
        

    def run(self):
        rospy.loginfo("action is starting in  seconds")
        Model = LSTMPredictor()
        Model.load_state_dict(torch.load("/home/btknzn/tiago_public_ws/src/batu_training/scripts/working.pth"))
        self.prepare_robot()
        #self.armMototionINT()
        while not rospy.is_shutdown():
            X1_MIN = 0.04995837
            X1_MAX = 0.7562845
            arm_joint1 = (self.arm_joint1-X1_MIN)/(X1_MAX-X1_MIN)
            X2_MIN = -0.43724102
            X2_MAX = 1.0187681
            arm_joint2 = (self.arm_joint2-X2_MIN)/(X2_MAX-X2_MIN)
            X3_MIN = -3.0051005
            X3_MAX = -0.7699633
            arm_joint3 = (self.arm_joint3-X3_MIN)/(X3_MAX-X3_MIN)

            X4_MIN = 0.5167487
            X4_MAX = 2.0607007
            arm_joint4 = (self.arm_joint4-X4_MIN)/(X4_MAX-X4_MIN)
            X5_MIN = -2.0750659
            X5_MAX = 2.0743227
            arm_joint5 = (self.arm_joint5-X5_MIN)/(X5_MAX-X5_MIN)

            X6_MIN = -1.3956555
            X6_MAX = 1.3958309
            arm_joint6 = (self.arm_joint6-X6_MIN)/(X6_MAX-X6_MIN)
            
            X8_MIN = 0.022458013
            X8_MAX = 0.045057185
            gripper_joint1 = (self.gripper_joint1-X8_MIN)/(X8_MAX-X8_MIN)

            X9_MIN = 0.022458013
            X9_MAX = 0.045057185
            gripper_joint2 = (self.gripper_joint2-X9_MIN)/(X9_MAX-X9_MIN)

            X7_MIN = -1.8210393
            X7_MAX = 2.0740623
            arm_joint7= (self.arm_joint7-X7_MIN)/(X7_MAX-X7_MIN)


            X10_MIN = -2.1165963e-06
            X10_MAX = 0.3500109
            torso_joint = (self.torso_joint-X10_MIN)/(X10_MAX-X10_MIN)

            X11_MIN = -0.00033559513
            X11_MAX = 0.0006243193
            torsohead_joint1 = (self.torsohead_joint1-X11_MIN)/(X11_MAX-X11_MIN)

            X12_MIN = -0.7685144
            X12_MAX = 0.0014590342  
            torsohead_joint2 = (self.torsohead_joint2-X12_MIN)/(X12_MAX-X12_MIN)
            ## Update predicted value via model:
            x_current = torch.asarray([arm_joint1,arm_joint2,arm_joint3,arm_joint4,arm_joint5,arm_joint6,arm_joint7,gripper_joint1,gripper_joint2,torso_joint,torsohead_joint1,torsohead_joint2]).reshape(1,12)


            #rospy.loginfo(self.cv_imageRGB_resized.shape)
            self.cv_imageRGB_resized.resize([self.cv_imageRGB_resized.shape[2],self.cv_imageRGB_resized.shape[0],self.cv_imageRGB_resized.shape[1]])

            image_current= torch.asarray(self.cv_imageRGB_resized)
            image_current = image_current[None,:]
            #rospy.loginfo("image_shape"+str(image_current.shape))
            self.cv_imageD_resized.resize(1,self.cv_imageD_resized.shape[0],self.cv_imageD_resized.shape[1])
            depth_current = torch.asarray(self.cv_imageD_resized)
            depth_current = depth_current[:,None,:,:]
            depth_current = depth_current.reshape(depth_current.shape[0],depth_current.shape[1],depth_current.shape[3],depth_current.shape[2])
            #rospy.loginfo("depth_shape"+str(depth_current.shape))
            x_current = x_current[None,:,:]
            #rospy.loginfo("input_shape"+str(x_current.shape))
            R_means = 188.3424
            R_std = 54.67417
            G_means = 177.37274
            G_std = 66.43382
            B_means = 169.08133
            B_std = 72.04574
            D_std =  1.165845
            D_means = 1.2093502
            normalized_image = TF.normalize(image_current.float(),[R_means, G_means, B_means], [R_std, G_std, B_std])
            normalized_disparity = TF.normalize(depth_current.float(),[D_means], [D_std])
            #rospy.loginfo("current status")
            #rospy.loginfo(x_current)
            #rospy.loginfo("normalized image")
            #rospy.loginfo(normalized_image)
            #rospy.loginfo("disparity")
            #rospy.loginfo(normalized_disparity)
            rospy.loginfo(x_current)
            x_next = Model(x_current.float(),normalized_image.float(),normalized_disparity.float())
 
            rospy.loginfo(x_next)
            rospy.loginfo(normalized_image)
            rospy.loginfo(normalized_disparity)
            #rospy.loginfo(str(x_next))
            #rospy.loginfo("running")


            x_next_shape = x_next.shape
            rospy.loginfo(x_next_shape)
            #rospy.loginfo("action from NN: {}".format(x_next_shape))
            #rospy.loginfo("action from NN:",+str(x_next.shape))
            X1_MIN = 0.04995837
            X1_MAX = 0.7562845
            X2_MIN = -0.43724102
            X2_MAX = 1.0187681
            X3_MIN = -3.0051005
            X3_MAX = -0.7699633
            X4_MIN = 0.5167487
            X4_MAX = 2.0607007
            X5_MIN = -2.0750659
            X5_MAX = 2.0743227
            X6_MIN = -1.3956555
            X6_MAX = 1.3958309
            X7_MIN = -1.8210393
            X7_MAX = 2.0740623
            X8_MIN = 0.022458013
            X8_MAX = 0.045057185
            X9_MIN = 0.022458013
            X9_MAX = 0.045057185
            X10_MIN = -2.1165963e-06
            X10_MAX = 0.3500109
            X11_MIN = -0.00033559513
            X11_MAX = 0.0006243193
            X12_MIN = -0.7685144
            X12_MAX = 0.0014590342 
            self.arm_joint1_precited = float((x_next[0,0].item()))*(X1_MAX-X1_MIN)+X1_MIN
            self.arm_joint2_precited = float((x_next[0,1].item()))*(X2_MAX-X2_MIN)+X2_MIN
            self.arm_joint3_precited = float((x_next[0,2].item()))*(X3_MAX-X3_MIN)+X3_MIN
            self.arm_joint4_precited = float((x_next[0,3].item()))*(X4_MAX-X4_MIN)+X4_MIN
            self.arm_joint5_precited = float((x_next[0,4].item()))*(X5_MAX-X5_MIN)+X5_MIN
            self.arm_joint6_precited = float((x_next[0,5].item()))*(X6_MAX-X6_MIN)+X6_MIN
            self.arm_joint7_precited = float((x_next[0,6].item()))*(X7_MAX-X7_MIN)+X7_MIN
            #rospy.loginfo(str(x_next))
            

            self.armMototion()
            self.gripper_joint1_predicted = float((x_next[0,7].item()))*(X8_MAX-X8_MIN)+X8_MIN
            self.gripper_joint2_predicted = float((x_next[0,8].item()))*(X9_MAX-X9_MIN)+X9_MIN
            self.action_gripper()
            self.torso_joint_predicted =  float((x_next[0,9].item()))*(X10_MAX-X10_MIN)+X10_MIN
            self.action_torso()
            self.torsohead_joint1_predicted = float((x_next[0,10].item()))*(X11_MAX-X11_MIN)+X11_MIN
            self.torsohead_joint2_predicted = float((x_next[0,11].item()))*(X12_MAX-X12_MIN)+X12_MIN
            rospy.loginfo(str(self.torsohead_joint1_predicted))
            self.action_head()
            rospy.sleep(1)

            
 


if __name__ == '__main__':
    try:
        agent = neuralNetwork()
        agent.run()

    except rospy.ROSInterruptException:
        pass