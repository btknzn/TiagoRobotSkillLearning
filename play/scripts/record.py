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
import os
import ros_numpy
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
arm_joint7s = []
gripper_joint1s = []
gripper_joint2s = []
torso_joints = []
torsohead_joint1s = []
torsohead_joint2s = [] 

class ThreadClass(QtCore.QThread):
    def __init__(self, parent = None):
        super(ThreadClass,self).__init__(parent)
        self.is_running = True
        self.ImageRGB = []
        self.ImageD = []
        self.arm_joint1 = []
        self.arm_joint2 = []
        self.arm_joint3 = []
        self.arm_joint4 = []
        self.arm_joint5 = []
        self.arm_joint6 = []
        self.arm_joint7 = []
        self.gripper_joint1 = []
        self.gripper_joint2 = []
        self.torso_joint = []
        self.torsohead_joint1 = []
        self.torsohead_joint2 = []



    def run(self):
        self.is_running = True
        while(self.is_running):
            rospy.loginfo("Saving joints")
            #movebases.append(self.cmd_vel)
            arm_joint1s.append(self.arm_joint1)
            arm_joint2s.append(self.arm_joint2)
            arm_joint3s.append(self.arm_joint3)
            arm_joint4s.append(self.arm_joint4)
            arm_joint5s.append(self.arm_joint5)
            arm_joint6s.append(self.arm_joint6)
            arm_joint7s.append(self.arm_joint7)
            imageRGBs.append(self.ImageRGB)
            imageDs.append(self.ImageD)
            gripper_joint1s.append(self.gripper_joint1)
            gripper_joint2s.append(self.gripper_joint2)
            torsohead_joint1s.append(self.torsohead_joint1)
            torsohead_joint2s.append(self.torsohead_joint2)
            torso_joints.append(self.torso_joint)

            rospy.sleep(0.1)
    def stop(self):
        self.is_running = False
        rospy.loginfo("Stopping Thread...")
        self.terminate()

    def setImage(self,ImageRGB):
        self.ImageRGB = ImageRGB
    def setImageD(self,ImageD):
        self.ImageD = ImageD
    def setArJoint(self,arm_joint1,arm_joint2,arm_joint3,arm_joint4,arm_joint5,arm_joint6,arm_joint7):
        self.arm_joint1 = arm_joint1
        self.arm_joint2 = arm_joint2
        self.arm_joint3 = arm_joint3
        self.arm_joint4 = arm_joint4
        self.arm_joint5 = arm_joint5
        self.arm_joint6 = arm_joint6
        self.arm_joint7 = arm_joint7

    def setGrippter(self,gripper_joint1,gripper_joint2 ):
        self.gripper_joint1 = gripper_joint1 
        self.gripper_joint2 = gripper_joint2

    def setTorso(self,torso_joint):
        self.torso_joint = torso_joint
    
    def setTorsoHead(self,torsohead_joint1, torsohead_joint2):
        self.torsohead_joint1 = torsohead_joint1
        self.torsohead_joint2 = torsohead_joint2





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
        self.k = 0
    
    

    def startRecording(self):
        rospy.loginfo("recording is started")
        self.threadclass.start()

    def saveRecording(self):
        #rospy.loginfo("recording is saved, as csv")
        dict = {  'arm_joint1': arm_joint1s, 'arm_joint2': arm_joint2s, 'arm_joint3': arm_joint3s,
        'arm_joint4': arm_joint4s, 'arm_joint5': arm_joint5s,'arm_joint6': arm_joint6s, 'arm_joint7':arm_joint7s, 'gripper_joint1':gripper_joint1s,'gripper_joint2':gripper_joint2s,'torso_joint':torso_joints,'torso_head1':torsohead_joint1s,'torso_head2':torsohead_joint2s}  
        df = pd.DataFrame(dict)

        df.to_csv('/home/btknzn/Desktop/dataexample/'+str(self.k)+'.csv')
        depth_folder = '/home/btknzn/Desktop/dataexample/depth/'+str(self.k)+'/'
        image_folder = '/home/btknzn/Desktop/dataexample/Image/'+str(self.k)+'/'
        self.k = self.k+1
        for i in range(0,len(arm_joint1s)):
            currentNumber = 'label'+str(i)
            img = np.array(imageRGBs[i])
            imgD = np.array(imageDs[i])
            rospy.loginfo(imgD)
            depth_path = depth_folder + currentNumber + str('.jpg')
            image_path = image_folder + currentNumber + str('.jpg')
            #rospy.loginfo(image_path)
            cv2.imwrite(image_path , img)
            cv2.imwrite(depth_path , imgD)
            #rospy.loginfo("images saved")


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
        self.bridge = CvBridge()
        self.image_subI = rospy.Subscriber("/xtion/rgb/image_color",Image,self.image_callback)
        self.image_subD = rospy.Subscriber("/xtion/depth_registered/image_raw",Image,self.imageD_callback)
        self.arm_listener = rospy.Subscriber("/arm_controller/state",JointTrajectoryControllerState,self.arm_callback)
        self.gripper_listener = rospy.Subscriber("/gripper_controller/state",JointTrajectoryControllerState,self.gripper_callback)
        self.torso_listener = rospy.Subscriber("/torso_controller/state",JointTrajectoryControllerState,self.torso_callback)
        self.torsohead_listener = rospy.Subscriber("/head_controller/state",JointTrajectoryControllerState,self.head_callback)

        self.pushButton_1.clicked.connect(self.startRecording)
        self.pushButton_2.clicked.connect(self.stopRecording)
        self.pushButton_3.clicked.connect(self.saveRecording)

    def torso_callback(self,torso_information):
        torso_joint = torso_information.actual.positions[0]
        self.threadclass.setTorso(torso_joint)

    def head_callback(self,torsohead_information):
        torsohead_joint1 = torsohead_information.actual.positions[0]
        torsohead_joint2 = torsohead_information.actual.positions[1]
        self.threadclass.setTorsoHead(torsohead_joint1,torsohead_joint2)
         

    def image_callback(self,ImageRGB):
        cv_imageRGB = ros_numpy.numpify(ImageRGB)
        down_width = 128
        down_height = 96
        down_points = (down_width, down_height)
        cv_imageRGB_resized = cv2.resize(cv_imageRGB,down_points)
        self.threadclass.setImage(cv_imageRGB_resized)

    
    def imageD_callback(self,ImageD):
        cv_imageD = ros_numpy.numpify(ImageD)
        mask = np.isnan(cv_imageD )
        cv_imageD [mask] = 0
        down_width = 128
        down_height = 96
        down_points = (down_width, down_height)
        cv_imageD_resized = cv2.resize(cv_imageD ,down_points,interpolation = cv2.INTER_CUBIC)
        #rospy.loginfo(cv_imageD_resized)
        self.threadclass.setImageD(cv_imageD_resized)



    def arm_callback(self,arm_information):
        #rospy.loginfo(arm_information.actual.positions[0])
        arm_joint1 = arm_information.actual.positions[0]
        arm_joint2 = arm_information.actual.positions[1]
        arm_joint3 = arm_information.actual.positions[2]
        arm_joint4 = arm_information.actual.positions[3]
        arm_joint5 = arm_information.actual.positions[4]
        arm_joint6 = arm_information.actual.positions[5]
        arm_joint7 = arm_information.actual.positions[6]
        self.threadclass.setArJoint(arm_joint1,arm_joint2,arm_joint3,arm_joint4,arm_joint5,arm_joint6,arm_joint7)
    def gripper_callback(self,gripper_information):
        gripper_joint1 = gripper_information.actual.positions[0]
        gripper_joint2 = gripper_information.actual.positions[1]
        self.threadclass.setGrippter(gripper_joint1,gripper_joint2)




        




if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    MainWindow.setWindowTitle("Recording Screeen")
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())