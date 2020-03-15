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

        # /最終敵位置トピックパブ用
        self.pub_last_enemy_pos=rospy.Publisher('enemy_pos_from_lider_last',Point,queue_size=1)
        self.last_enemy_pos=Point(0,1.3,0)

        # /敵位置マーカ
        self.marker=Marker()
        self.marker.header.frame_id="map"
        self.marker.ns = "basic_shapes"
        self.marker.id = 0
        self.marker.scale.x=self.marker.scale.y=self.marker.scale.z=0.20
        self.marker.color.a=1.0
        self.marker.color.r=1.0
        self.marker.type=Marker.CUBE
        self.marker.action = Marker.ADD
        self.enemy_marker_pub = rospy.Publisher('enemy_pos_from_lider_marker',Marker,queue_size=1)

        # /最終敵位置マーカ
        self.last_marker=Marker()
        self.last_marker.header.frame_id="map"
        self.last_marker.ns = "basic_shapes"
        self.last_marker.id = 0
        self.last_marker.scale.x=self.last_marker.scale.y=self.last_marker.scale.z=0.20
        self.last_marker.color.a=1.0
        self.last_marker.color.b=1.0
        self.last_marker.type=Marker.CUBE
        self.last_marker.action = Marker.ADD
        self.last_marker.pose.position.x= 1.3
        self.last_marker.pose.position.y= 0
        self.enemy_last_marker_pub = rospy.Publisher('enemy_pos_from_lider_last_marker',Marker,queue_size=1)

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
                #障害物の半径が10センチ以上か
                if(obs.radius>=0.10):
                    continue
                #センターオブジェクトか
                if(abs(enemy_pos.x) <=0.350+judge_enemy_mergin and abs(enemy_pos.y) <=0.350+judge_enemy_mergin):
                #    print("is_center",enemy_pos)
                    continue
                #コーナーオブジェクトか
                if((abs(enemy_pos.x) >=0.420-judge_enemy_mergin and abs(enemy_pos.x) <=0.640+judge_enemy_mergin) and \
                   (abs(enemy_pos.y) >=0.445-judge_enemy_mergin and abs(enemy_pos.y) <=0.615+judge_enemy_mergin)):
                #    print("is_corner",enemy_pos)
                    continue
                #壁か
                if((abs(enemy_pos.y)+abs(enemy_pos.x)) >=1.650-judge_enemy_mergin):
                #    print("is_wall",enemy_pos)
                    continue


                self.pub_enemy_pos.publish(enemy_pos)
                self.last_enemy_pos=enemy_pos                

                #敵位置マーカー
                self.marker.pose.position=obs.center
                self.marker.header.stamp = rospy.Time.now()
                self.marker.id = 1
                self.marker.color.r=1.0
                self.marker.color.b=0.0                
                self.marker.lifetime=rospy.Duration(0.5)
                self.enemy_marker_pub.publish(self.marker)
                self.last_marker=self.marker

            self.pub_last_enemy_pos.publish(self.last_enemy_pos)

            #最終敵位置マーカー
            self.last_marker.id = 2
            self.last_marker.color.r=0.0
            self.last_marker.color.b=1.0
            self.enemy_last_marker_pub.publish(self.last_marker)
            r.sleep()



def main(args):
    rospy.init_node('enemy_pos_from_lider', anonymous=True)
    ra = enemy_pos_from_lider()
    # print('[enemy_pos_from_lider]initialized')
    ra.run()

if __name__=='__main__':
    main(sys.argv)
