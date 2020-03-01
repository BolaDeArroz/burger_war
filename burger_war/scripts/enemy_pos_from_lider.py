#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import math
import numpy as np

import roslib
import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan,PointCloud2,PointCloud
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from burger_war.msg import MyPose
import tf
from PIL import Image
import os 
import cv2
from laser_geometry import LaserProjection
from obstacle_detector.msg import Obstacles





class enemy_pos_from_lider:
    def __init__(self):
        # bot name 
        robot_name=''
        self.name = robot_name

        # /Obstaclesトピックサブスクライブ用
        self.obstacles=Obstacles()
        self.obstacles_sub = rospy.Subscriber('/{}/obstacles'.format(self.name), Obstacles, self.obstacle_callback)

        # /敵位置トピックパブ用
        self.pub_enemy_pos=rospy.Publisher('enemy_pos_from_lider',Point,queue_size=1)

 
        self.marker=Marker()

        self.marker.header.frame_id="map"
        self.marker.ns = "basic_shapes"
        self.marker.id = 0
        self.marker.scale.x=self.marker.scale.y=self.marker.scale.z=0.25
        self.marker.color.a=1.0
        self.marker.color.r=1.0
        self.marker.type=Marker.CUBE
        self.marker.action = Marker.ADD
        self.enemy_marker_pub = rospy.Publisher('enemy_pos_from_lider_marker',Marker,queue_size=1)


    def obstacle_callback(self, data):
        self.obstacles=data


    def run(self):
        r=rospy.Rate(5)
        while not rospy.is_shutdown():
            obstacles=self.obstacles
            for obs in obstacles.circles:
                enemy_pos=Point()
                #横軸x,縦軸yの座標に戻す
                enemy_pos.x=-obs.center.y
                enemy_pos.y= obs.center.x
                


                #敵とオブジェクトを見分けるマージン[m]。値が大きいほど、オブジェクトだと判定するエリアが大きくなる。
                judge_enemy_mergin=0.0
                #フィルタリング
                #センターオブジェクトか
                if(abs(enemy_pos.x) <=0.350+judge_enemy_mergin and abs(enemy_pos.y) <=0.350+judge_enemy_mergin):
                    continue
                #コーナーオブジェクトか
                if((abs(enemy_pos.y) >=0.420-judge_enemy_mergin and abs(enemy_pos.y) <=0.640+judge_enemy_mergin) and \
                   (abs(enemy_pos.y) >=0.445-judge_enemy_mergin and abs(enemy_pos.y) <=0.615+judge_enemy_mergin)):
                    continue
                #壁か
                if((abs(enemy_pos.y)+abs(enemy_pos.x)) >=1.500-judge_enemy_mergin):
                    continue

                self.marker.pose.position=obs.center
                self.marker.header.stamp = rospy.Time.now()
                self.marker.id = 1
                self.marker.lifetime=rospy.Duration(0.5)
                self.enemy_marker_pub.publish(self.marker)
                self.pub_enemy_pos.publish(enemy_pos)
            r.sleep()



def main(args):
    rospy.init_node('enemy_pos_from_lider', anonymous=True)
    ra = enemy_pos_from_lider()
    print('[enemy_pos_from_lider]initialized')
    ra.run()

if __name__=='__main__':
    main(sys.argv)
