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

        self.enemy_potential_array=np.zeros((240,240))#10mm毎

    def obstacle_callback(self, data):
        self.obstacles=data

    def enemy_pos_move_avg(self,x,y,potential):
        margin=5 #margin+-50mm 発見地の周辺何mmまで確率付与するか

        x_idx_max=self.enemy_potential_array.shape[0]-1
        y_idx_max=self.enemy_potential_array.shape[1]-1

        rot_x=x*math.cos(math.radians(45))-y*math.sin(math.radians(45))
        rot_y=x*math.sin(math.radians(45))+y*math.cos(math.radians(45))
        rot_x=(rot_x+1.2)*100 #m->10mm
        rot_y=(rot_y+1.2)*100 #m->10mm
        if(rot_x>=x_idx_max):rot_x=self.enemy_potential_array[0]-1
        elif(rot_x<=0):rot_x=0
        if(rot_y>=y_idx_max):rot_y=self.enemy_potential_array[1]-1
        elif(rot_y<=0):rot_y=0
        x_start=int(0 if rot_x-margin<=  0 else rot_x-margin)
        x_end  =int(x_idx_max-1 if rot_x+margin>=x_idx_max-1 else rot_x+margin)
        y_start=int(0 if rot_y-margin<=  0 else rot_y-margin)
        y_end  =int(y_idx_max-1 if rot_y+margin>=y_idx_max-1 else rot_y+margin)
        self.enemy_potential_array[x_start:x_end,y_start:y_end]=self.enemy_potential_array[x_start:x_end,y_start:y_end]+potential #+確率

        max_idx=np.unravel_index(np.argmax(self.enemy_potential_array),self.enemy_potential_array.shape)
        print(self.enemy_potential_array[max_idx])
        if (self.enemy_potential_array[max_idx]>=100):
            origin_x=float(max_idx[0])/100-1.2
            origin_y=float(max_idx[1])/100-1.2
            #print(origin_x,origin_y)
            ori_x=origin_x*math.cos(math.radians(-45))-origin_y*math.sin(math.radians(-45))
            ori_y=origin_x*math.sin(math.radians(-45))+origin_y*math.cos(math.radians(-45))
             #print(ori_x,ori_y)
            return True,ori_x,ori_y
        return False,0,0


    def run(self):

        r=rospy.Rate(5)


        while not rospy.is_shutdown():
#            self.object_marker_pub.publish(self.object_marker)
            obstacles=self.obstacles
            for obs in obstacles.circles:
                enemy_pos=Point()
                #横軸x,縦軸yの座標に戻す
                enemy_pos.x=-obs.center.y
                enemy_pos.y= obs.center.x
                
                #敵とオブジェクトを見分けるマージン[m]。値が大きいほど、オブジェクトだと判定するエリアが大きくなる。
                radius_mergin=0.0#半径
                center_mergin=0.15#センター
                cornar_mergin=0.2#コーナー
                wall_mergin=0.05#壁
                potential=80#敵確率初期値
                
                #フィルタリング
                #障害物の半径が10センチ以上か
                if(obs.radius>=0.10-radius_mergin):
                    continue
                elif(obs.radius>=0.10):
                    potential=50
                    
                #センターオブジェクトか
                if(abs(enemy_pos.x) <=0.175+center_mergin and abs(enemy_pos.y) <=0.175+center_mergin):
                    continue
                elif(abs(enemy_pos.x) <=0.175 and abs(enemy_pos.y) <=0.175 ):
                    potential=30

                #コーナーオブジェクトか
                if((abs(enemy_pos.x) >=0.430-cornar_mergin and abs(enemy_pos.x) <=0.640+cornar_mergin) and \
                   (abs(enemy_pos.y) >=0.455-cornar_mergin and abs(enemy_pos.y) <=0.605+cornar_mergin)):
                    continue
                elif((abs(enemy_pos.x) >=0.430 and abs(enemy_pos.x) <=0.640) and \
                   (abs(enemy_pos.y) >=0.455 and abs(enemy_pos.y) <=0.605)):
                    potential=30
                #壁か(2400*ルート2/2=1.697)
                if((abs(enemy_pos.y)+abs(enemy_pos.x)) >=1.697-wall_mergin):
                #    print("is_wall",enemy_pos)
                    continue
                elif((abs(enemy_pos.y)+abs(enemy_pos.x)) >=1.697):
                    potential=50
                
                is_enemy_ext,x,y=self.enemy_pos_move_avg(enemy_pos.x,enemy_pos.y,potential)

                if is_enemy_ext:
                    self.pub_enemy_pos.publish(Point(x,y,0))
                    self.last_enemy_pos=enemy_pos                

                    #敵位置マーカー
                    self.marker.pose.position=obs.center
                    self.marker.header.stamp = rospy.Time.now()
                    self.marker.id = 1
                    self.marker.color.r=1.0
                    self.marker.color.b=0.0                
                    self.marker.lifetime=rospy.Duration(0.1)
                    self.enemy_marker_pub.publish(self.marker)
                    self.last_marker=self.marker

            self.enemy_potential_array=self.enemy_potential_array*0.7#減衰
            self.enemy_potential_array=self.enemy_potential_array.clip(0,100)
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
