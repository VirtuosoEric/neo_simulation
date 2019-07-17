#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import Twist
# from pub_n_sub.msg import Num

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rotation = False

def two_callback(data):
    global pub,rotation
    if rotation:
        pub.publish(data)
        print 'rotation 2'

def three_callback(data):
    global pub,rotation
    if not rotation:
        pub.publish(data)
        print 'no rotation 3'

def rotation_switch(msg):
    global rotation

    if msg.data:
        rotation = True
    else:
        rotation = False

    


if __name__ == '__main__':
    rospy.init_node('rotation_switch', anonymous=False)
    rospy.Subscriber("cmd_vel_2", Twist, two_callback)
    rospy.Subscriber("cmd_vel_3", Twist, three_callback)
    rospy.Subscriber("rotation",Bool,rotation_switch)
    rospy.spin()
    # listener()