#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from geometry_msgs.msg import Twist

import tf


import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
from nav_msgs.msg import Path


# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo as CamInfoMSG
from cv_bridge import CvBridge, CvBridgeError
import cv2
import copy
import image_geometry
import numpy as np
import math

from image_function import get_tracking_info
from calc_motion_planning import rotation_operate, check_possession_marker



class AttackBot():
    def __init__(self):
        # bot name 
        robot_name=rospy.get_param('~robot_name')
        self.name = robot_name
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # lidar scan subscriber
        self.scan = LaserScan()
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidar_callback)

        # camera subscribver
        # for convert image topic to opencv obj
        self.image = None
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/{}/image_raw'.format(self.name), Image, self.image_callback)
        self.camerainfo_sub = rospy.Subscriber("/{}/camera_info".format(self.name),CamInfoMSG, self.camerainfo_callback)

        #warstate subscriber
        self.war_state = None
        self.war_state_sub = rospy.Subscriber('/{}/war_state'.format(self.name), String, self.warstate_callback)

        # test nuvirun
        self.current_global_plan = Path()
        rospy.Subscriber("/"+self.name+"/move_base/DWAPlannerROS/global_plan", Path, self.callback)

    

    # lidar scan topic call back sample
    # update lidar scan state
    def lidar_callback(self, data):
        self.scan = data

    def warstate_callback(self, data):
        self.war_state = data
        #print(self.war_state)

    # camera image call back sample
    # comvert image topic to opencv object and show
    def image_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.original_image = copy.copy(self.image)
        except CvBridgeError as e:
            print("***********************", e)

        """
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.original_image, "bgr8"))
        except CvBridgeError as e:
            print('CV_Bridge_Error')
        """

    def camerainfo_callback(self,data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

    # 


    def set_goal(self, x, y, yaw):
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
        # recovery
        self.recoverMove(goal)
        #wait = self.client.wait_for_result()
        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        


    def attack_war(self):
        TRACKING_MODE = False
        r = rospy.Rate(5) # change speed 5fps
        listener = tf.TransformListener()
        now = rospy.Time.now()
        timeout_dur = 20  # default time out
        # start time log
        start_time = now
        #listener.waitForTransform('/' + self.name +"/map",self.name +"/base_link", rospy.Time(),rospy.Duration(4.0))   
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            # change strategy by possession marker info 
            obtain_marker_list = check_possession_marker(self.war_state)
            if '_L' in obtain_marker_list and '_R' in obtain_marker_list:
                # TODO
                timeout_dur = 30
            # time out
            if now.secs - start_time.secs > timeout_dur:
                print('Time Out!!!')
                self.client.cancel_goal()
                break
            # change strategy by tracking info 
            tracking_info = get_tracking_info(self.image)
            print(tracking_info)
            # deciede to navigation or tracking
            if tracking_info != {}:
                print('Find enemy')
                # continue navigation
                if tracking_info['target'] == 'red_ball':
                    pass
                elif tracking_info['target'] == 'green_side' or tracking_info['target'] == 'burger':
                    if TRACKING_MODE == False:
                        self.client.cancel_goal()
                        print('stop navigation')
                    TRACKING_MODE = True
            else:
                if TRACKING_MODE == True:
                    # missing enemy
                    print('Missing enemy')
                    """
                    self.client.cancel_goal()
                    """
                    break
                    
                TRACKING_MODE = False
            # change navigation to tracking
            if TRACKING_MODE is False:
                result = self.set_goal(0.7,0.35,-3.1415/4)
                #print('++++++++++++++++++', result)
            else:
                command = 99
                mu_x = tracking_info['center'][0] 
                if tracking_info['target'] == 'burger':
                    command = 0
                else:
                    # tracking enemy
                    if mu_x > 370:
                        command = 1
                    elif mu_x < 270:
                        command = 2
                twist = rotation_operate(command)
                self.vel_pub.publish(twist)
            
            """
            # get own local coordinate
            listener.waitForTransform(self.name +"/map",self.name +"/base_link", now, rospy.Duration(4.0))
            position, quaternion = listener.lookupTransform(self.name +"/map", self.name +"/base_link", rospy.Time())
            print(position, quaternion)
            """
            r.sleep()


    def callback(self,data):
        self.current_global_plan=data
        rospy.loginfo(str(data.poses[0].pose.position.x))

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
                if ((1/(moved_dist+0.01))*elapsed_time_sec)>=20 and rotated_rad <=0.5:
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
                    a_position, a_orientation = listener.lookupTransform(self.name +"/map", self.name +"/base_link", rospy.Time())
                    rospy.loginfo("a_position.x="+str(a_position[0])+"a_position.y="+str(a_position[1]))
                    rospy.loginfo("middle_pose.x="+str(middle_pose.position.x)+"middle_pose.y="+str(middle_pose.position.y))
                    rospy.loginfo("mod_middle_pose.x="+str(-middle_pose.position.y)+"mod_middle_pose.y="+str(middle_pose.position.x))

                    x1=a_position[0]
                    y1=a_position[1]
                    x2=middle_pose.position.y
                    y2=-middle_pose.position.x
                    #中間地点向きの角度算出
                    rad=math.atan2(y2-y1,x2-x1)
                    rospy.loginfo(str(rad))
                    #角度をマップ座標系に変換
                    map_rad=rad + a_orientation[2]
                    print(map_rad, 'ssssssssssssssssssss')
                    print(a_orientation[2])
                    #現在位置からnavigation(回転)
                    self.set_goal(x1,y1,map_rad)

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

        

if __name__ == '__main__':
    rospy.init_node('attackrun')
    bot = AttackBot()
    bot.attack_war()

