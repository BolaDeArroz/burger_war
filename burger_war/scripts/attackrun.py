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
        """
        #wait = self.client.wait_for_result()
        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        
        """


    def attack_war(self, enemey_position_x, enemey_position_y, enemey_position_yaw):
        TRACKING_MODE = False
        r = rospy.Rate(30) # change speed 30fps
        listener = tf.TransformListener()
        now = rospy.Time.now()
        timeout_dur = 20  # default time out
        # start time log
        start_time = now
        #listener.waitForTransform('/' + self.name +"/map",self.name +"/base_link", rospy.Time(),rospy.Duration(4.0))   
        while not rospy.is_shutdown():
            # failed move_base
            if self.client.get_state() == actionlib_msgs.msg.GoalStatus.ABORTED:
                print('ABORTED')
                recovery_abort()
            elif self.client.get_state() == actionlib_msgs.msg.GoalStatus.REJECTED:
                print('REJECTED')
                self.client.cancel_goal()
                recovery_reject()
                break
            elif self.client.get_state() == actionlib_msgs.msg.GoalStatus.SUCCEEDED:
                print('SUCCEEDED')
            """
            elif self.get_state == actionlib_msgs.msg.GoalStatus.LOST:
                recover_lost()
            """
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
            if self.image is None:
                continue
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
                result = self.set_goal(enemey_position_x, enemey_position_y, enemey_position_yaw)
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
        pub_vel(self,vel_x=-0.15,vel_y=0,vel_z=0,time_sec=1.0)
        #停止
        pub_vel(self,vel_x=0,vel_y=0,vel_z=0,time_sec=0.5)

    def recovery_reject():
        # TODO
        pass

        
        

if __name__ == '__main__':
    rospy.init_node('attackrun')
    bot = AttackBot()
    bot.attack_war()

