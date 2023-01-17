#!/usr/bin/env python
from PyQt5 import QtCore, QtGui, QtWidgets
import rospy
from batu_training.msg import objectLocation
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import tf 
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped
from std_msgs.msg import Header
import rospy, tf2_ros, geometry_msgs.msg
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
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
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

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

import numpy as np
from std_srvs.srv import Empty

import cv2
from cv_bridge import CvBridge



from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
	if not name[:1] == '_':
		code = MoveItErrorCodes.__dict__[name]
		moveit_error_dict[code] = name


shifBeforeObject = +0.35
shifTable = +0.4

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix


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
        self.pushButton_4.setObjectName("pushButton_4")
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

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    
    def lift_torso(self):
        rospy.loginfo("Moving torso up")
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.34]
        jtp.time_from_start = rospy.Duration(2.5)
        jt.points.append(jtp)
        self.torso_cmd.publish(jt)
    
    def lower_head(self):
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0, -0.99]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)

    def normal_head(self):
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0, 0.0]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)
    
    def lower_torso(self):
        rospy.loginfo("Moving torso down")
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0]
        jtp.time_from_start = rospy.Duration(2.5)
        jt.points.append(jtp)
        self.torso_cmd.publish(jt)

    def graspPosition(self):
        rospy.loginfo("hand Adjustment")
        jt = JointTrajectory()
        jt.joint_names = ['arm_1_joint','arm_2_joint','arm_3_joint','arm_4_joint','arm_5_joint','arm_6_joint','arm_7_joint' ]
        jtp = JointTrajectoryPoint()
        jtp.positions = [1.82 , -1.32, -3.46, 0.04, -1.57 , 1.37, 0.00]
        jtp.time_from_start = rospy.Duration(2.5)
        jt.points.append(jtp)
        self.arm_cmd.publish(jt)
        

    def prepare_robot(self):
        rospy.loginfo("Unfold arm safely")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'pregrasp'
        pmg.skip_planning = False
        self.play_m_as.send_goal_and_wait(pmg)
        rospy.loginfo("Done.")
        rospy.loginfo("Robot prepared.")


    def moveToRed(self):
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        rospy.loginfo("Going to :" + str(self.goal_red))
        client.send_goal(self.goal_red)
        wait = client.wait_for_result()
        rospy.sleep(1.0)
        rospy.loginfo("Movetored completed")

    def getReadyGrasp(self):
        rospy.sleep(2.0)
        self.prepare_robot()
        rospy.sleep(2.0)
        self.lower_head()
        #self.lower_torso()

    
    def redBottle_callback(self,msg):
        self.goal_red.target_pose.pose.orientation.w = 1.0
        camera_location = np.array( [msg.x,msg.y,msg.z])
        (self.trans, self.rot) = self.tf.lookupTransform('map','xtion_depth_optical_frame',  rospy.Time(0))
        rot_matrix = quaternion_rotation_matrix(self.rot)
        location = np.matmul(rot_matrix,camera_location) +self.trans
        self.goal_red.target_pose.pose.position.x  = location[0]-shifBeforeObject 
        self.goal_red.target_pose.pose.position.y  = location[1]
        self.goal_red.target_pose.pose.position.z  = location[2]*0
        self.pick_pose.pose.position.x = msg.x
        self.pick_pose.pose.position.y = msg.y
        self.pick_pose.pose.position.z = msg.z
        self.pick_pose.pose.orientation.x = 0
        self.pick_pose.pose.orientation.y = 0
        self.pick_pose.pose.orientation.z = 0
        self.pick_pose.pose.orientation.w = 1

    
    def blueArea_callback(self,msg_blue):
        self.goal_blue.target_pose.pose.orientation.w = 1.0
        camera_location = np.array( [msg_blue.x,msg_blue.y,msg_blue.z])
        (self.trans, self.rot) = self.tf.lookupTransform('map','xtion_depth_optical_frame',  rospy.Time(0))
        rot_matrix = quaternion_rotation_matrix(self.rot)
        location = np.matmul(rot_matrix,camera_location) +self.trans
        self.goal_blue.target_pose.pose.position.x  = location[0]-shifTable
        self.goal_blue.target_pose.pose.position.y  = location[1]
        self.goal_blue.target_pose.pose.position.z  = location[2]*0
        self.place_pose.pose.position.x = msg_blue.x-shifTable/2
        self.place_pose.pose.position.y = msg_blue.y
        self.place_pose.pose.position.z = msg_blue.z
        self.place_pose.pose.orientation.x = 0
        self.place_pose.pose.orientation.y = 0
        self.place_pose.pose.orientation.z = 0
        self.place_pose.pose.orientation.w = 1
        
        

    def pickObject(self):
        ps = PoseStamped()
        ps.pose.position.x = self.pick_pose.pose.position.x
        ps.pose.position.y = self.pick_pose.pose.position.y
        ps.pose.position.z = self.pick_pose.pose.position.z
        ps.pose.orientation.x = 0
        ps.pose.orientation.y = 0
        ps.pose.orientation.z = 0
        ps.pose.orientation.w = 1
        transform = self.tfBuffer.lookup_transform("base_footprint", 
									   "xtion_depth_optical_frame",
									   rospy.Time(0))
        object_ps = do_transform_pose(ps, transform)
        self.pick_g.object_pose.pose.position = object_ps.pose.position
        self.pick_g.object_pose.header.frame_id = 'base_footprint'
        self.pick_g.object_pose.pose.orientation.w = 1.0
        self.pick_g.object_pose.pose.position.z = self.pick_g.object_pose.pose.position.z 
        self.pick_g.object_pose.pose.position.x = self.pick_g.object_pose.pose.position.x+0.02
        #self.graspPosition()
        rospy.sleep(2.0)
        self.detected_pose_pub.publish(self.pick_g.object_pose)
        rospy.loginfo("Gonna pick:" + str(self.pick_g))
        #self.graspPosition()
        self.pick_as.send_goal_and_wait(self.pick_g)
        rospy.loginfo("Done!")
        #self.lift_torso()
        rospy.loginfo("getting ready for goint to aim")
        self.lift_torso()
        self.normal_head()
        rospy.loginfo("picking action is completed")
    


    def placeObject(self):
        self.place_as.send_goal_and_wait(self.pick_g)
        rospy.loginfo("object placed")


    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Action Window"))
        self.pushButton_1.setText(_translate("MainWindow", "Approach to RED OBJECT"))
        self.pushButton_2.setText(_translate("MainWindow", "Pick Red Object"))
        self.pushButton_3.setText(_translate("MainWindow", "get Ready for grasp"))
        self.pushButton_4.setText(_translate("MainWindow", " PLACE object back"))
        self.label.setText(_translate("MainWindow", "ROBOT MOTIONS"))
        rospy.init_node("robot_control")
        self.tf = tf.TransformListener()
        self.subscriberRedObject = rospy.Subscriber("redBottle", objectLocation, self.redBottle_callback)
        self.subscriberBlueArea = rospy.Subscriber("blueArea", objectLocation, self.blueArea_callback)
        self.pick_pose = PoseStamped()
        self.place_pose = PoseStamped()
        self.goal_red = MoveBaseGoal()
        self.goal_red.target_pose.header.frame_id = "map"
        self.goal_red.target_pose.header.stamp = rospy.Time.now()
        self.goal_blue = MoveBaseGoal()
        self.goal_blue.target_pose.header.frame_id = "map"
        self.goal_blue.target_pose.header.stamp = rospy.Time.now()
        self.pushButton_1.clicked.connect(self.moveToRed)
        self.pushButton_2.clicked.connect(self.pickObject)
        self.pushButton_3.clicked.connect(self.getReadyGrasp)
        self.pushButton_4.clicked.connect(self.placeObject)
        self.bridge = CvBridge()
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_l = tf2_ros.TransformListener(self.tfBuffer)
        self.pick_as = SimpleActionClient('/pickup_pose', PickUpPoseAction)
        self.place_as = SimpleActionClient('/place_pose', PickUpPoseAction)
        self.torso_cmd = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1)
        self.head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
        self.arm_cmd = rospy.Publisher('/arm_controller/command',JointTrajectory,  queue_size=1)
        self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
        self.detected_pose_pub = rospy.Publisher('/detected_aruco_pose',
							 PoseStamped,
							 queue_size=1,
							 latch=True)
        self.pick_g = PickUpPoseGoal()




if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())