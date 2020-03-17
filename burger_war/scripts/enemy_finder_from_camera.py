#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
import roslib
import copy
import math
import csv

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from burger_war.msg import MyPose 
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3

from image_function import Normalization, detect_enemy_robot


class EnemyFinderFromCamera():
    def __init__(self):
        # bot name 
        robot_name=''
        self.name = robot_name
        # camera info
        self.horizontal_angle = 60

        self.bridge = CvBridge()
        # pub
        self.img_pub = rospy.Publisher("enemy_finder", Image, queue_size=1)
        self.enemy_pose_pub=rospy.Publisher('enemy_pose_from_camera',MyPose,queue_size=1)
        # sub
        self.image = None
        self.img_sub = rospy.Subscriber("/{}/image_raw".format(self.name), Image, self.image_callback)
        self.my_pose = None
        self.my_pose_sub = rospy.Subscriber('/{}/my_pose'.format(self.name), MyPose, self.my_pose_callback)

        self.theta= 0
        self.ds= 0

    
    def my_pose_callback(self,data):
        self.my_pose = data
        # print('my_pose', self.my_pose)


    def image_callback(self, data):
        try:
            self.image = data
        except CvBridgeError as e:
            print("image_callback error: ", e)


    def calc_enemy_local_coordinate(self, size, width, x_center):
        r = size / 2

        area = r * r * math.pi

        #ds = (-1.1492 * area + 1891.2) / 1000 - 0.1
        ds = -0.00137 * area + 1.28286
        x_center_pixel = self.x_trans_image_coordinate(x_center, width)
        radian_pixel = float(self.horizontal_angle)/float(width)
        th = (x_center_pixel * radian_pixel) * math.pi/180
        self.theta = th
        self.ds = area
        """
        print('x_center_pixel', x_center_pixel)
        print('radian_pixel', radian_pixel)
        print('th', th)
        """
        x = ds * math.cos(th)
        y = ds * math.sin(th)

        return [x, y]

    def x_trans_image_coordinate(self, origin_c,  image_width):
        trans_result = origin_c - image_width/2
        return trans_result




    def calc_enemy_pose(self):
        coordinate_list = []
        try:
            image = Normalization(self.bridge.imgmsg_to_cv2(self.image, "bgr8"))
            enemy = detect_enemy_robot(image)
            pub_array = Float32MultiArray()

            h, w, _ = image.shape

            if enemy['red_ball'] != []:
                c, r = cv2.minEnclosingCircle(enemy['red_ball'][0])

                # array.data = [1, c[0] - (w / 2.0), 2 * r, w, h]
                coordinate_list = self.calc_enemy_local_coordinate(r, w, c[0])
                """
                print('radias :', r)
                print('center', c)
                print('x, y', coordinate_list)
                """
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except Exception as e:
            print('calc enemy pose error', e)
        
        return coordinate_list

        

    def run(self):
        r=rospy.Rate(5)
        """
        path = '/home/ctu/rhc_branch_ws/src/burger_war/position_logs.csv'
        with open(path, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z', 'th', 'ds'])
        """
        while not rospy.is_shutdown():
            enemy_coordinate_list = self.calc_enemy_pose()
            try:
                # print('my_pose',  self.my_pose.pos)
                """
                with open(path, 'a') as f:
                    writer = csv.writer(f)
                    writer.writerow([self.my_pose.pos.x, self.my_pose.pos.y, self.my_pose.pos.z, self.theta, self.ds])
                """
                if enemy_coordinate_list != []:
                    # 
                    try:
                        enemy_pose_x = self.my_pose.pos.x + enemy_coordinate_list[0]
                        enemy_pose_y = self.my_pose.pos.y + enemy_coordinate_list[1]
                    except Exception as e:
                        print('RUN error: ', e)
                    #
                    enemy_pose = MyPose(pos           =Point(enemy_pose_x , enemy_pose_y, self.my_pose.pos.z),\
                           ori_quaternion=Quaternion( 0, 0, 0,0),\
                           ori_euler     =Vector3(            0,         0, 0)          )
                    """
                    print('enemy_pose',  enemy_pose.pos)
                    print('my_pose',  self.my_pose.pos)
                    """
                    self.enemy_pose_pub.publish(enemy_pose)
            except Exception as e:
                print('run error: ', e)
            
            
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("enemy_finder_from_camera")
    finder = EnemyFinderFromCamera()
    finder.run()
    rospy.spin()
