#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import math
import roslib
import rospy
import smach
import smach_ros
import tf
import actionlib
import actionlib_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal



def setGoal(move_base_client,x,y,yaw):
    move_base_client.wait_for_server()
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x =  y
    goal.target_pose.pose.position.y = -x

    # Euler to Quartanion
    q=tf.transformations.quaternion_from_euler(0,0,yaw-math.pi/2)        
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    move_base_client.send_goal(goal)


