#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool,Float32MultiArray
import tf


import actionlib
import actionlib_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2


class NaviBot():
    def __init__(self):
        # bot name 
        robot_name=rospy.get_param('~robot_name')
        self.name = robot_name
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        
        self.current_global_plan = Path()

        rospy.Subscriber("/"+self.name+"/move_base/DWAPlannerROS/global_plan", Path, self.global_plan_callback)
        #rospy.Subscriber("/"+self.name+"/move_base/NavfnROS/plan", Path, self.callback)
        
        self.is_enemy_detected=False
        self.array=Float32MultiArray()
        rospy.Subscriber("/"+self.name+"/array", Float32MultiArray, self.enemy_detect_callback)


    def enemy_detect_callback(self,array):
        print("EnemyDetect", array.data[0], self.is_enemy_detected)	   
        if array.data[2] >= 11 and int(array.data[0]!=0):
            self.is_enemy_detected = True
        self.array=array

    def global_plan_callback(self,data):
        self.current_global_plan=data
        rospy.loginfo(str(data.poses[0].pose.position.x))

    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.name + "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)

        #wait = self.client.wait_for_result()
        #if not wait:
        #    rospy.logerr("Action server not available!")
        #    rospy.signal_shutdown("Action server not available!")
        #else:
        #    return self.client.get_result()  

    def pub_vel(self,vel_x=0,vel_y=0,vel_z=0,time_sec=0.5):
        cmd_vel=Twist()
        cmd_vel.linear.x=vel_x
        cmd_vel.linear.y=vel_y
        cmd_vel.linear.z=vel_z
        end_time_sec=float(rospy.Time.now().secs+rospy.Time.now().nsecs/(1000*1000*1000)+time_sec)
        while not rospy.is_shutdown():
            rospy.loginfo("cmd_vel.x="+str(cmd_vel.linear.x)+"cmd_vel.y="+str(cmd_vel.linear.y))
            self.vel_pub.publish(cmd_vel)
            if end_time_sec < float(rospy.Time.now().secs+rospy.Time.now().nsecs/(1000*1000*1000)):
                break
            else:
                rospy.sleep(0.1)

    def recovery_abort(self):   
        #一定距離バック
        self.pub_vel(-0.15,0,0,0.8)
        #停止
        self.pub_vel(0,0,0,0.5)

    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps
        listener = tf.TransformListener()
        try:
            listener.waitForTransform(self.name +"/map",self.name +"/base_link",rospy.Time(),rospy.Duration(4.0))
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr("tf_err")
        listener.waitForTransform(self.name +"/map",self.name +"/base_link",rospy.Time(),rospy.Duration(4.0))
        

        #現在位置計算
        cur_position, _ = listener.lookupTransform(self.name +"/map", self.name +"/base_link", rospy.Time())
        #cur_eular=tf.transformations.euler_from_quaternion(cur_orientation)

        #中継地点リスト
        # waypoints = [
        #     [-1.1, 0.3, 3.1415/4],
        #     [-0.7, 0.7, 3.1415/4],
        #     #[-0.7, 0.6, 0],
        #     [-0.4, 1.0, 0],
        #     #[0.0, 1.4,-3.1415/4],#左角(初期位置から)
        #     [ 0.4, 1.0,-3.1415/4],
        #     [ 0.8, 0.60,-3.1415/4],
        #     [ 1.1, 0.3,-3.1415/4*2],
        #     #[ 1.25, 0.0,-3.1415/4*3],#対角(初期位置から)
        #     [ 1.1 ,-0.3,-3.1415/4*3],
        #     [ 0.7 ,-0.7,-3.1415/4*3],
        #     [ 0.4 ,-1.0,3.1415],
        #     #[ 0.0 ,-1.4,3.1415/4*3],#右角(初期位置から)
        #     [-0.4 ,-1.0,3.1415/4*3],
        #     [-0.7 ,-0.7,3.1415/4*3],
        #     [-0.9 ,-0.4,3.1415/4*2]
        # ]

        waypoints = [
            [-0.96,0.27,3.1415/4],
            [-0.81,0.40,3.1415/4],
            [-0.36,0.83,0],
            [0.44,0.90,-3.1415/4],
            [0.85,0.47,-3.1415/4*2],
            [0.94,-0.33,-3.1415/4*3],
            [0.45,-0.87,-3.1415],
            [-0.46,-0.91,3.1415/4*3],
            [-0.86,-0.47,3.1415/4*2]
        ]

        ###############################
        #while not rospy.is_shutdown():
        #    r.sleep()
        ###################################
        #最近地点計算
        nearest_waypoint_idx=0
        nearest_distance = 10
        for (idx_waypoint,waypoint) in enumerate(waypoints):
            tmp_distance=math.sqrt((cur_position[0]-waypoint[0])**2 + (cur_position[1]-waypoint[1])**2 )
            if tmp_distance <nearest_distance:
                nearest_distance=tmp_distance
                nearest_waypoint_idx=idx_waypoint

        #最近地点からナビゲーション開始
        next_waypoint_idx=nearest_waypoint_idx
        self.is_enemy_detected=False
        while not self.is_enemy_detected:
            waypoint=waypoints[next_waypoint_idx]
            rospy.loginfo(str(next_waypoint_idx))
            self.setGoal(waypoint[0],waypoint[1],waypoint[2])
            rospy.loginfo("state="+str(self.client.get_state()))
            print("is_enemy_detected",self.is_enemy_detected)
            while self.client.get_state() in [actionlib_msgs.msg.GoalStatus.ACTIVE,actionlib_msgs.msg.GoalStatus.PENDING] and not self.is_enemy_detected:
                now = rospy.Time.now()
                listener.waitForTransform(self.name +"/map",self.name +"/base_link", now, rospy.Duration(4.0))

                # map座標系の現在位置をｔｆから取得する
                position, quaternion = listener.lookupTransform(self.name +"/map", self.name +"/base_link", rospy.Time())

                # ウェイポイントのゴールの周囲0.25ｍ以内にロボットが来たら、次のウェイポイントを発行する
                if(math.sqrt((position[0]-waypoint[0])**2 + (position[1]-waypoint[1])**2 ) <= 0.25):
                    print "next!!"
                    if next_waypoint_idx+1 >= len(waypoints):
                        next_waypoint_idx=0
                    else:
                        next_waypoint_idx+=1
                    break
                r.sleep()

            if self.client.get_state()==actionlib_msgs.msg.GoalStatus.SUCCEEDED:
                    print "next!!"
                    if next_waypoint_idx+1 >= len(waypoints):
                        next_waypoint_idx=0
                    else:
                        next_waypoint_idx+=1
                    continue


            if self.client.get_state()==actionlib_msgs.msg.GoalStatus.ABORTED:
                self.recovery_abort()
                self.client.cancel_goal()
                
            if self.is_enemy_detected:
                rospy.loginfo("enemy_detected!!")
                self.client.cancel_goal()
                break
            r.sleep()
        self.client.cancel_goal()
        return self.array.data[1], self.array.data[2]

if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.strategy()


