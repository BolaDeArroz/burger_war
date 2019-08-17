#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf


import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

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

        rospy.Subscriber("/"+self.name+"/move_base/DWAPlannerROS/global_plan", Path, self.callback)
        #rospy.Subscriber("/"+self.name+"/move_base/NavfnROS/plan", Path, self.callback)



    def callback(self,data):
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
        self.recoverMove(goal)

        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()  

          

    def recoverMove(self,goal):
        
        rospy.loginfo("recover")
        #現在地点取得用リスナー
        listener = tf.TransformListener()
        listener.waitForTransform(self.name +"/map",self.name +"/base_link", rospy.Time(), rospy.Duration(4.0))
        # map座標系の現在位置をｔｆから取得し、前地点に設定
        pre_position, pre_orientation = listener.lookupTransform(self.name +"/map", self.name +"/base_link", rospy.Time())
        pre_time=rospy.Time.now()
        rospy.sleep(0.5)
        
        rospy.loginfo("fuga")
        #loop
        while self.client.get_state()==actionlib.SimpleGoalState.ACTIVE:
            rospy.loginfo(self.client.get_state())
            #map座標系の現在位置をｔｆから取得し、現在地点に設定
            cur_position, cur_orientation = listener.lookupTransform(self.name +"/map", self.name +"/base_link", rospy.Time())
            #移動距離計算
            moved_dist = math.sqrt((cur_position[0]-pre_position[0])**2 + (cur_position[1]-pre_position[1])**2 )
            rospy.loginfo("moved_dist="+str(moved_dist))

            #角度差分計算
            rotated_rad = math.fabs(cur_orientation[2]-pre_orientation[2])
            rospy.loginfo("rotated_rad="+str(rotated_rad))
            #移動距離が一定値(m)を下回る場合
            if moved_dist <= 0.5:
                #経過時間計算
                cur_time=rospy.Time.now()
                rospy.loginfo("cur_time"+str(cur_time.secs)+"."+str(cur_time.nsecs))
                rospy.loginfo("pre_time"+str(pre_time.secs)+"."+str(pre_time.nsecs))
                elapsed_time_sec=float((cur_time.secs+cur_time.nsecs/(1000*1000*1000))-(pre_time.secs+pre_time.nsecs/(1000*1000*1000)))
                rospy.loginfo("elapsed_time_sec="+str(elapsed_time_sec))
                rospy.loginfo("1/(moved_dist+0.01)*elapsed_time_sec="+str(1/(moved_dist+0.01)*elapsed_time_sec))
                #1/移動距離*経過時間がしきい値を超える場合(要計算)
                if ((1/(moved_dist+0.01))*elapsed_time_sec)>=40 and rotated_rad <=0.5:
                    #現在のplan保存
                    current_plan=self.current_global_plan

                    rospy.loginfo("robot is stucked") 
                    #send_goal actionキャンセル
                    self.client.cancel_goal()
                    rospy.loginfo("current goal is canceled")

                    
                    #一定距離バック
                    rospy.loginfo("back")
                    back_cmd=Twist()
                    back_cmd.linear.x=-0.15
                    back_cmd.linear.y=0
                    back_cmd.angular.z=0
                    end_time_sec=float(rospy.Time.now().secs+rospy.Time.now().nsecs/(1000*1000*1000)+1.0)
                    while not rospy.is_shutdown():
                        rospy.loginfo("linear.x="+str(back_cmd.linear.x)+"linear.y="+str(back_cmd.linear.y))
                        self.vel_pub.publish(back_cmd)
                        if end_time_sec < float(rospy.Time.now().secs+rospy.Time.now().nsecs/(1000*1000*1000)):
                            break
                        else:
                            rospy.sleep(0.1)
                    #停止
                    rospy.loginfo("stop")
                    stop_cmd=Twist()
                    stop_cmd.linear.x=0
                    stop_cmd.linear.y=0
                    stop_cmd.angular.z=0
                    self.vel_pub.publish(stop_cmd)
                    rospy.sleep(0.5)

                    #global_planから中間地点の座標を得る
                    #rospy.loginfo(str(self.current_global_plan.poses[0].pose.position.x))
                    #middle_pose=current_plan.poses[int(math.floor(len(current_plan.poses)/2))].pose
                    middle_pose=current_plan.poses[0].pose

                    #中間地点方向に回転===================
                    #map座標系の現在位置をｔｆから取得し、現在地点に設定
                    listener.waitForTransform(self.name +"/map",self.name +"/base_link", rospy.Time(), rospy.Duration(4.0))
                    a_position, _ = listener.lookupTransform(self.name +"/map", self.name +"/base_link", rospy.Time())
                    rospy.loginfo("a_position.x="+str(a_position[0])+"a_position.y="+str(a_position[1]))
                    rospy.loginfo("middle_pose.x="+str(middle_pose.position.x)+"middle_pose.y="+str(middle_pose.position.y))
                    rospy.loginfo("mod_middle_pose.x="+str(-middle_pose.position.y)+"mod_middle_pose.y="+str(middle_pose.position.x))

                    x1=a_position[0]
                    y1=a_position[1]
                    x2=-middle_pose.position.y
                    y2=middle_pose.position.x
                    #中間地点向きの角度算出
                    rad=math.atan2(y2-y1,x2-x1)
                    rospy.loginfo(str(rad))
                    #角度をマップ座標系に変換
                    map_rad=-rad
                    #現在位置からnavigation(回転)
                    self.setGoal(x1,y1,map_rad)

                    #中間地点をゴールに設定
                    rospy.loginfo("set_middle_goal")
                    middle_goal = MoveBaseGoal()
                    middle_goal.target_pose.header.frame_id = self.name + "/odom"
                    middle_goal.target_pose.header.stamp = rospy.Time.now()
                    middle_goal.target_pose.pose=middle_pose
                    rospy.loginfo("middle_goal_ori_x="+str(middle_goal.target_pose.pose.orientation.x))
                    rospy.loginfo("middle_goal_ori_y="+str(middle_goal.target_pose.pose.orientation.y))
                    rospy.loginfo("middle_goal_ori_z="+str(middle_goal.target_pose.pose.orientation.z))
                    rospy.loginfo("middle_goal_ori_w="+str(middle_goal.target_pose.pose.orientation.w))                    

                    #self.client.send_goal(middle_goal)
                    #self.recoverMove(middle_goal)

                    #経過時間リセット
                    pre_time=rospy.Time.now()
                    #元のゴールに再設定
                    self.client.send_goal(goal)
                    self.recoverMove(goal)
                    #map座標系の現在位置をｔｆから取得し、前地点座標更新
                    pre_position, pre_orientation = listener.lookupTransform(self.name +"/map", self.name +"/base_link", rospy.Time())
            #移動距離が一定値を下回らない場合
            else:
                #経過時間リセット
                pre_time=rospy.Time.now()
                #前地点座標更新
                pre_position=cur_position
                pre_orientaion=cur_orientation

            #sleep
            rospy.sleep(0.5)

    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps
        listener = tf.TransformListener()
        try:
            listener.waitForTransform(self.name +"/map",self.name +"/base_link",rospy.Time(),rospy.Duration(4.0))
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr("tf_err")
        listener.waitForTransform(self.name +"/map",self.name +"/base_link",rospy.Time(),rospy.Duration(4.0))
        #中継地点リスト
        waypoints = [
            [-1.1, 0.3, 3.1415/4],
            [-0.7, 0.7, 3.1415/4],
            [-0.4, 1.0, 0],
            #[0.0, 1.4,-3.1415/4],#左角(初期位置から)
            [ 0.4, 1.0,-3.1415/4],
            [ 0.7, 0.7,-3.1415/4],
            [ 1.1, 0.3,-3.1415/4*2],
            #[ 1.25, 0.0,-3.1415/4*3],#対角(初期位置から)
            [ 1.1 ,-0.3,-3.1415/4*3],
            [ 0.7 ,-0.7,-3.1415/4*3],
            [ 0.4 ,-1.0,3.1415],
            #[ 0.0 ,-1.4,3.1415/4*3],#右角(初期位置から)
            [-0.4 ,-1.0,3.1415/4*3],
            [-0.7 ,-0.7,3.1415/4*3],
            [-1.1 ,-0.3,3.1415/4*2]
        ]
        #self.setGoal(-1.15,0.0,3.1415/4)

        for waypoint in waypoints:
            self.setGoal(waypoint[0],waypoint[1],waypoint[2])
            if waypoint[0] == -0.4:
                break
            #while not rospy.is_shutdown():
            #     now = rospy.Time.now()
            #     listener.waitForTransform(self.name +"/map",self.name +"/base_link", now, rospy.Duration(4.0))

            #     # map座標系の現在位置をｔｆから取得する
            #     position, quaternion = listener.lookupTransform(self.name +"/map", self.name +"/base_link", rospy.Time())

            #     # ウェイポイントのゴールの周囲0.25ｍ以内にロボットが来たら、次のウェイポイントを発行する
            #     if(math.sqrt((position[0]-waypoint[0])**2 + (position[1]-waypoint[1])**2 ) <= 0.25):
            #         print "next!!"
            #         break

            #     else:
            #         rospy.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.strategy()