#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('mobile_base_controller/cmd_vel', Twist)
    rospy.init_node('test', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    move_cmd = Twist()
    move_cmd.linear.x = 1.0
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(move_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass