#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rotation = False
got_first_goal = False

goal_pose = Pose
goal_x = 0
goal_y = 0

def normal_cmd(data):
    global pub,rotation
    if rotation:
        pub.publish(data)
        print 'rotation'

def no_rotation(data):
    global pub,rotation
    if not rotation:
        pub.publish(data)
        print 'no rotation'

def rotation_switch(msg):
    global rotation

    if msg.data:
        rotation = True
    else:
        rotation = False

def update_goal(msg):
    global goal_x,goal_y,got_first_goal
    goal_pose = msg.goal.target_pose.pose
    goal_x = goal_pose.position.x
    goal_y = goal_pose.position.y
    got_first_goal = True
    print goal_pose

def goal_dist(msg):
    global goal_x,goal_y,rotation
    if got_first_goal:
        robot_pose = msg.feedback.base_position.pose
        dist = math.sqrt((goal_x - robot_pose.position.x)**2+(goal_y - robot_pose.position.y)**2)
        if dist < 0.3:
            rotation = True
        else:
            rotation = False
    

if __name__ == '__main__':
    rospy.init_node('rotation_switch', anonymous=False)
    rospy.Subscriber("cmd_vel_normal", Twist, normal_cmd)
    rospy.Subscriber("cmd_vel_no_rotation", Twist, no_rotation)
    rospy.Subscriber("rotation",Bool,rotation_switch)
    rospy.Subscriber('/move_base/feedback',MoveBaseActionFeedback,goal_dist)
    rospy.Subscriber('/move_base/goal',MoveBaseActionGoal,update_goal)
    rospy.spin()