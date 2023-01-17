#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from batu_training.msg import objectLocation
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import tf 
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped
import tf2_ros, tf2_geometry_msgs
from std_msgs.msg import Header
import rospy, tf2_ros, geometry_msgs.msg
from geometry_msgs.msg import PointStamped
from scipy.spatial.transform import Rotation as R
import numpy as np


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


class NumberCounter:
    def __init__(self):
        self.number_subscriber = rospy.Subscriber("redBottle", objectLocation, self.callback_number)
        self.tf = tf.TransformListener()


    def callback_number(self, msg):
        rospy.loginfo("coordinate: "+str(msg.x)+" "+str(msg.y)+ " "+str(msg.z))
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = msg.x
        goal.target_pose.pose.position.y = msg.y
        goal.target_pose.pose.position.z = msg.z
        goal.target_pose.pose.orientation.w = 1.0
        #client.send_goal(goal)
        #wait = client.wait_for_result()
        #if not wait:
        #    rospy.logerr("Action server not available!")
        #    rospy.signal_shutdown("Action server not available!")
        #else:
        #    rospy.loginfo(str(client.get_result()))
        (trans, rot) = self.tf.lookupTransform('map','xtion_depth_optical_frame',  rospy.Time(0))
        camera_location = np.array( [msg.x,msg.y,msg.z])

        print("camera location: ")
        print(camera_location)
        print("rotation Matrix quarterion")
        print(rot)
        print("translation matrix")
        print(trans)
        rot_matrix = quaternion_rotation_matrix(rot)
        print("rotation matrix")
        print(rot_matrix)
        location = np.matmul(rot_matrix,camera_location) +trans
        goal.target_pose.pose.position.x  = location[0]-0.2
        goal.target_pose.pose.position.y  = location[1]-0.2
        goal.target_pose.pose.position.z  = location[2]-0.2
        print("transformed location:")
        print(location)
        client.send_goal(goal)
        wait = client.wait_for_result()
        
        

    
if __name__ == '__main__':
    rospy.init_node('gotobottle')
    NumberCounter()
    rospy.spin()