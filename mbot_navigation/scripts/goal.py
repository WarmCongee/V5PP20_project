#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs import PoseStamped

# Callbacks definition

#def active_cb(extra):
    #rospy.loginfo("Goal pose being processed")

#def feedback_cb(feedback):
    #rospy.loginfo("Current location: "+str(feedback))

#def done_cb(status, result):
    #if status == 3:
        #rospy.loginfo("Goal reached")
    #if status == 2 or status == 8:
        #rospy.loginfo("Goal cancelled")
    #if status == 4:
        #rospy.loginfo("Goal aborted")
    

rospy.init_node('send_goal')
while 1 :
    navclient = rospy.Publisher('move_base_simple/goal',PoseStamped,queue_size=1)

    # Example of navigation goal
    goal = PoseStamped()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = 8
    goal.target_pose.pose.position.y = 10
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.662
    goal.target_pose.pose.orientation.w = 0.750

    navclient.publish(goal)


