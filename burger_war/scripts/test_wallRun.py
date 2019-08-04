#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import random
import time
import sys
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallRunBot():
    
    def __init__(self, bot_name):
        # bot name 
        self.name = bot_name
    
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        self.latest_scan=LaserScan()
        self.laser_sub=rospy.Subscriber("scan", LaserScan, self.laser_callback)
        self.cmd_vel=Twist()

    def laser_callback(self,data):
        self.latest_scan=data

    def strategy(self):
        r = rospy.Rate(10)

        left_prev=sys.float_info.max
        
        while not rospy.is_shutdown():
            if len(self.latest_scan.ranges) > 0:
                #LaserScanメッセージをすでに受け取っている場合
                front = sys.float_info.max
                left = sys.float_info.max
                #theta-range 座標系から x-y 座標系に変換
                for (_,i_range) in zip(range(22),self.latest_scan.ranges):
                    if not(i_range < self.latest_scan.range_min or \
                        i_range > self.latest_scan.range_max or \
                        i_range is None):

                        #距離値がエラーでない場合
                        print("i_range="+str(i_range))
                        theta = self.latest_scan.angle_min + i_range *self.latest_scan.angle_increment
                        #x-y 座標系に変換
                        x =i_range*math.cos(theta)
                        y=i_range* math.sin(theta)

                        if math.fabs(y)<0.25 and x > 0.05:
                            #ロボット正面方向のデータについて最小距離を計算
                            if front > x :
                                front =x
                        elif math.fabs(x) <0.25 and y>0.25:
                            if left > y:
                                left=y
                if front > 0.15:
                    #正面の距離に余裕がある場合
                    print("Following left wall (distance"+str(left)+")")
                    
                    self.cmd_vel.linear.x = 0.1

                    if left >1.0:
                        left=1.0
					    
                    #角速度指令値を0に近づけるようにフィードバック

                    self.cmd_vel.angular.z += -self.cmd_vel.angular.z * 0.01
					#左方向の距離を0.5mに近づけるようにフィードバック
                    self.cmd_vel.angular.z += (left - 0.5) * 0.02
                    
					#距離の変化量(壁の向きを表す)を0に近づけるようにフィードバック
                    if left_prev < 1.0:
                        self.cmd_vel.angular.z += (left - left_prev) * 4.0
                    
                    if self.cmd_vel.angular.z > 0.3:
                        self.cmd_vel.angular.z = 0.3
                    
                    elif self.cmd_vel.angular.z < -0.3:
                        self.cmd_vel.angular.z = -0.3
                else:
                    print("Something in front")
                    print(front)
                    self.cmd_vel.linear.x=0.0
                    self.cmd_vel.angular.z=-0.2
                
                self.vel_pub.publish(self.cmd_vel)

            r.sleep()

if __name__ == '__main__':
    rospy.init_node('random_ccr')
    bot = WallRunBot("wall")
    bot.strategy()

