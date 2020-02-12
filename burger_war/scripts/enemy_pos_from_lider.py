#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import math
import roslib
import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
#from visualization_msgs.msg import Marker
import tf
class enemy_pos_from_lider:
    def __init__(self):
        # bot name 
        robot_name=rospy.get_param('~robot_name')
        self.name = robot_name

        #s
        # self.scan_ranges=[]
        # self.scan_sub = rospy.Subscriber('/{}/scan'.format(self.name), LaserScan, self.scan_callback)

        #self.global_occpacy_grid=[]
        #self.map_sub=rospy.Subscriber('/{}/move_base/global_costmap/costmap'.format(self.name), OccupancyGrid, self.global_costmap_callback)

        # self.marker=Marker()
        # self.marker.header.frame_id="/"+self.name+"/map"
        # self.marker.ns = "basic_shapes"
        # self.marker.id = 0
        # self.marker.scale.x=self.marker.scale.y=self.marker.scale.z=0.1
        # self.marker.color.a=1.0        
        # self.marker.type=Marker.CUBE
        # self.marker.action = Marker.ADD
        # self.marker_pub = rospy.Publisher('enemy_marker',Marker,queue_size=1)

        #self.enemy_is_back_pub=rospy.Publisher('is_enemy_back',Poibt,queue_size=1)

    def scan_callback(self, data):
        self.scan_ranges=data.ranges

    #def global_costmap_callback(self,data):
    #    self.global_occpacy_grid=data

    def run(self):
        r=rospy.Rate(5)
        while not rospy.is_shutdown():
            # my_eular=tf.transformations.euler_from_quaternion(my_orientation)
            # total_chance=0.0
            # for rader_rad in range(120,240):
            #     if self.scan_ranges[rader_rad] <1.0:
            #         rader_pos_x=my_position[0]+self.scan_ranges[rader_rad]*math.cos(my_eular[2]+math.pi*2*rader_rad/360)
            #         rader_pos_y=my_position[1]+self.scan_ranges[rader_rad]*math.sin(my_eular[2]+math.pi*2*rader_rad/360)
            #         self.marker.pose.position.x=rader_pos_x
            #         self.marker.pose.position.y=rader_pos_y
            #         self.marker.header.stamp = rospy.Time.now()
            #         self.marker.id = rader_rad
            #         one_chance,self.marker.color.b,self.marker.color.g,self.marker.color.r=self.calc_chance_of_enemy(rader_pos_x,rader_pos_y)
            #         total_chance+=one_chance
            #         self.marker.lifetime=rospy.Duration()
            #     else:
            #         self.marker.color.b=self.marker.color.g=self.marker.color.r=0.0
            #     self.marker_pub.publish(self.marker)       
            # print('[rader_find_enemy]total_chance=',total_chance)
            # if total_chance >10:
            #     self.enemy_is_back_pub.publish(True)
            # else:
            #     self.enemy_is_back_pub.publish(False)
            r.sleep()

    # def calc_chance_of_enemy(self,x,y):
    #     chance=0.0
    #     g=0.0
    #     b=0.0
    #     r=0.0
    #     #out of field
    #     if((math.fabs(x)+math.fabs(y))>1.5):
    #         chance=0.0
    #     elif(math.fabs(x)<0.3 and math.fabs(y)<0.3):#inside of center ofstacle
    #         chance=0.0
    #         g=1.0
    #     elif((math.fabs(x)<0.805 and math.fabs(x)>0.305) and (math.fabs(y)<0.730 and math.fabs(y)>0.330)):#inside of other ofstacle
    #         chance=0.0
    #         b=1.0
    #     else:
    #         chance=1.0
    #         r=1.0
    #     return chance,b,g,r
        
def main(args):
    rospy.init_node('rader_find_enemy', anonymous=True)
    ra = enemy_pos_from_score()
    print('[enemy_pos_from_score]initialized')
    ra.run()

if __name__=='__main__':
    main(sys.argv)
