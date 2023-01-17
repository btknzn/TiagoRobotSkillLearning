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
import matplotlib.image
# seed random number generator
movebases = []
movebasesLinearX = []
movebasesLinearY = []
movebasesLinearZ = []
movebasesAngularX = []
movebasesAngularY = []
movebasesAngularZ = []
imageRGBs = []
imageDs = []
arm_joint1s = []
arm_joint2s = []
arm_joint3s = []
arm_joint4s = []
arm_joint5s = []
arm_joint6s = []
gripper_joint1s = []
gripper_joint2s = []
torso_joints = []
torsohead_joint1s = []
torsohead_joint2s = [] 


class ThreadClass(QtCore.QThread):
    def __init__(self, parent = None):
        super(ThreadClass,self).__init__(parent)
        self.is_running = True
        self.cmd_vel_Lx = 0
        self.cmd_vel_Ly = 0
        self.cmd_vel_Lz = 0
        self.cmd_vel_Ax = 0
        self.cmd_vel_Ay = 0
        self.cmd_vel_Az = 0
        self.ImageRGB = []
        self.ImageD = []
        self.arm_joint1 = []
        self.arm_joint2 = []
        self.arm_joint3 = []
        self.arm_joint4 = []
        self.arm_joint5 = []
        self.arm_joint6 = []
        self.gripper_joint1 = []
        self.gripper_joint2 = []
        self.torso_joint = []
        self.torsohead_joint1 = []
        self.torsohead_joint2 = []
        self.k = 0


    def run(self):
        self.is_running = True
        while(self.is_running):
            rospy.loginfo("Saving joints")
            #movebases.append(self.cmd_vel)
            movebasesLinearX.append(self.cmd_vel_Lx)
            movebasesLinearY.append(self.cmd_vel_Ly)
            movebasesLinearZ.append(self.cmd_vel_Lz)
            movebasesAngularX.append(self.cmd_vel_Ax)
            movebasesAngularY.append(self.cmd_vel_Ay)
            movebasesAngularZ.append(self.cmd_vel_Az)
            arm_joint1s.append(self.arm_joint1)
            arm_joint2s.append(self.arm_joint2)
            arm_joint3s.append(self.arm_joint3)
            arm_joint4s.append(self.arm_joint4)
            arm_joint5s.append(self.arm_joint5)
            arm_joint6s.append(self.arm_joint6)
            imageRGBs.append(self.ImageRGB)
            imageDs.append(self.ImageD)
            gripper_joint1s.append(self.gripper_joint1)
            gripper_joint2s.append(self.gripper_joint2)
            torso_joints.append(self.torso_joint)
            torsohead_joint1s.append(self.torsohead_joint1)
            torsohead_joint2s.append(self.torsohead_joint2)
            rospy.sleep(0.1)
    def stop(self):
        self.is_running = False
        rospy.loginfo("Stopping Thread...")
        self.terminate()
    def setcmdVel(self,cmd_vel_input_lx,cmd_vel_input_ly, cmd_vel_input_lz, cmd_vel_input_Ax, cmd_vel_input_Ay, cmd_vel_input_Az):
        self.cmd_vel_Lx = cmd_vel_input_lx
        self.cmd_vel_Ly = cmd_vel_input_ly
        self.cmd_vel_Lz = cmd_vel_input_lz
        self.cmd_vel_Ax = cmd_vel_input_Ax
        self.cmd_vel_Ay = cmd_vel_input_Ay
        self.cmd_vel_Az = cmd_vel_input_Az
    def setImage(self,ImageRGB):
        self.ImageRGB = ImageRGB
    def setImageD(self,ImageD):
        self.ImageD = ImageD
    def setArJoint(self,arm_joint1,arm_joint2,arm_joint3,arm_joint4,arm_joint5,arm_joint6):
        self.arm_joint1 = arm_joint1
        self.arm_joint2 = arm_joint2
        self.arm_joint3 = arm_joint3
        self.arm_joint4 = arm_joint4
        self.arm_joint5 = arm_joint5
        self.arm_joint6 = arm_joint6

    def setGrippter(self,gripper_joint1,gripper_joint2 ):
        self.gripper_joint1 = gripper_joint1 
        self.gripper_joint2 = gripper_joint2

    def setTorso(self,torso_joint):
        self.torso_joint = torso_joint
    
    def setTorsoHead(self,torsohead_joint1,torsohead_joint2):
        self.torsohead_joint1= torsohead_joint1
        self.torsohead_joint2= torsohead_joint2




class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(130, 150, 531, 231))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.pushButton_2 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_2.setObjectName("pushButton_2")
        self.gridLayout.addWidget(self.pushButton_2, 0, 1, 1, 1)
        self.pushButton_1 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_1.setObjectName("pushButton_1")
        self.gridLayout.addWidget(self.pushButton_1, 0, 0, 1, 1)
        self.pushButton_3 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_3.setObjectName("pushButton_3")
        self.gridLayout.addWidget(self.pushButton_3, 1, 0, 1, 1)
        self.pushButton_4 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_4.setObjectName(" ")
        self.gridLayout.addWidget(self.pushButton_4, 1, 1, 1, 1)
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(140, 130, 521, 16))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setObjectName("label")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.threadclass = ThreadClass()
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
    
    

    def startRecording(self):
        rospy.loginfo("recording is started")
        self.threadclass.start()

    def saveRecording(self):
        rospy.loginfo("recording is saved, as csv")
        dict = { 'movebaseLinearX': movebasesLinearX , 'movebaseLinearY': movebasesLinearY, 'movebaseLinearZ': movebasesLinearZ, 
        'movebaseAngularX': movebasesAngularX , 'movebaseAngularY': movebasesAngularY, 'movebaseAngularZ': movebasesAngularZ,
        'image':imageRGBs,'imageDs':imageDs, 'arm_joint1': arm_joint1s, 'arm_joint2': arm_joint2s, 'arm_joint3': arm_joint3s,
        'arm_joint4': arm_joint4s, 'arm_joint5': arm_joint5s,'arm_joint6': arm_joint6s, 'gripper_joint1':gripper_joint1s,'gripper_joint2':gripper_joint2s,
        'torso_joint':torso_joints,'torsohead_joint1s':torsohead_joint1s,'torsohead_joints2s':torsohead_joint2s}  
        df = pd.DataFrame(dict)
        df.to_csv('/home/btknzn/Desktop/dataexample/Motion'+str(self.k)+'.csv')

        depth_folder= str('/home/btknzn/Desktop/dataexample/DepthImage/'+str(self.k)+"/")
        image_folder = str('/home/btknzn/Desktop/dataexample/Image/'+str(self.k)+"/")
        for i in range(0,len(movebasesLinearX)):
            currentNumber = 'label'+str(i)
            img = np.array(imageRGBs[i])
            imgD = np.array(imageDs[i])
            depth_path = depth_folder +"depthImage"+ currentNumber + str('.jpg')
            image_path = image_folder +"Image"+ currentNumber + str('.jpg')
            cv2.imwrite(depth_path , img)
            cv2.imwrite(image_path , imgD)
            rospy.loginfo("images saved")

        self.k = self.k+1
        rospy.loginfo(str(movebases))
        

    
    def stopRecording(self):
        rospy.loginfo("recording is stopped")
        self.threadclass.stop()
    

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Record Window"))
        self.pushButton_1.setText(_translate("MainWindow", "Start recording"))
        self.pushButton_2.setText(_translate("MainWindow", "Stop recording"))
        self.pushButton_3.setText(_translate("MainWindow", "Save recording"))
        rospy.init_node("recording_node")
        self.subscriberRedObject = rospy.Subscriber('mobile_base_controller/cmd_vel', Twist, self.movebase_callback)
        self.bridge = CvBridge()
        self.image_subI = rospy.Subscriber("/xtion/rgb/image_color",Image,self.image_callback)
        self.image_subD = rospy.Subscriber("/xtion/depth_registered/image_raw",Image,self.imageD_callback)
        self.arm_listener = rospy.Subscriber("/arm_controller/state",JointTrajectoryControllerState,self.arm_callback)
        self.gripper_listener = rospy.Subscriber("/gripper_controller/state",JointTrajectoryControllerState,self.gripper_callback)
        self.torso_listener = rospy.Subscriber('/torso_controller/state',JointTrajectoryControllerState,self.torso_callback)
        self.torsohead_listener = rospy.Subscriber('/head_controller/state',JointTrajectoryControllerState,self.torsohead_callback)
        self.pushButton_1.clicked.connect(self.startRecording)
        self.pushButton_2.clicked.connect(self.stopRecording)
        self.pushButton_3.clicked.connect(self.saveRecording)

    def image_callback(self,ImageRGB):
        cv_imageRGB = self.bridge.imgmsg_to_cv2(ImageRGB, "bgr8")
        down_width = 128
        down_height = 96
        down_points = (down_width, down_height)
        cv_imageRGB_resized = cv2.resize(cv_imageRGB,down_points)
        self.threadclass.setImage(cv_imageRGB_resized)

    
    def imageD_callback(self,ImageD):
        cv_imageD = self.bridge.imgmsg_to_cv2(ImageD, "8UC1")
        down_width = 128
        down_height = 96
        down_points = (down_width, down_height)
        cv_imageD_resized = cv2.resize(cv_imageD,down_points,interpolation = cv2.INTER_CUBIC)
        #rospy.loginfo(cv_imageD_resized)
        self.threadclass.setImageD(cv_imageD_resized)



    def movebase_callback(self,movebase_msg):
        self.threadclass.setcmdVel(movebase_msg.linear.x,movebase_msg.linear.y,movebase_msg.linear.z,movebase_msg.angular.x,movebase_msg.angular.y,movebase_msg.angular.z)
        #rospy.loginfo("girdi")
        #rospy.loginfo(str(movebase))

    def arm_callback(self,arm_information):
        #rospy.loginfo(arm_information.actual.positions[0])
        arm_joint1 = arm_information.actual.positions[0]
        arm_joint2 = arm_information.actual.positions[1]
        arm_joint3 = arm_information.actual.positions[2]
        arm_joint4 = arm_information.actual.positions[3]
        arm_joint5 = arm_information.actual.positions[4]
        arm_joint6 = arm_information.actual.positions[5]
        self.threadclass.setArJoint(arm_joint1,arm_joint2,arm_joint3,arm_joint4,arm_joint5,arm_joint6)
    def gripper_callback(self,gripper_information):
        gripper_joint1 = gripper_information.actual.positions[0]
        gripper_joint2 = gripper_information.actual.positions[1]
        self.threadclass.setGrippter(gripper_joint1,gripper_joint2)

    def torso_callback(self,torso_information):
        torso_joint = torso_information.actual.positions[0]
        self.threadclass.setTorso(torso_joint)
    def torsohead_callback(self,torsohead_information):
        torsohead_joint1 = torsohead_information.actual.positions[0]
        torsohead_joint2 = torsohead_information.actual.positions[1]
        self.threadclass.setTorsoHead(torsohead_joint1,torsohead_joint2)



        




if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    MainWindow.setWindowTitle("Recording Screeen")
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())