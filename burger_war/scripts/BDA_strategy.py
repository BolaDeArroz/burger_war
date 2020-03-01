#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import json
import rospy
import math


from std_msgs.msg import Float32MultiArray, Time, String, Bool, Int32MultiArray
from geometry_msgs.msg import Point
from burger_war.msg import MyPose 



class BDA_strategy():
    def __init__(self):
        # bot name 
        robot_name=''
        self.name = robot_name
        self.all_state_list = ['attack', 'escape', 'disturb']
        # sub
        """
        use almost all the our publish data
        - enemy_pos_from_score
        - pub_score
        - enemy_pos_from_lider
        - rem_time
        - my_pose
        - enemy_pose_from_camera
        """
        self.sub_enemy_pos_from_score = rospy.Subscriber('/{}/enemy_pos_from_score'.format(self.name), Float32MultiArray, self.enemy_pos_from_score_callback)
        self.enemy_pos_from_score=Float32MultiArray()

        self.sub_score = rospy.Subscriber('/{}/score'.format(self.name),Int32MultiArray,self.score_callback)
        self.score = []

        self.sub_enemy_pos_from_lider = rospy.Subscriber('/{}/enemy_pos_from_lider'.format(self.name), Point, self.enemy_pos_from_lider_callback)
        self.enemy_pos_from_lider={"enemy_pos":Point(),"is_topic_receive":False}
        
        self.sub_rem_time=rospy.Subscriber('rem_time',Time,self.rem_time_callback)
        self.rem_time = Time()

        self.sub_my_pose = rospy.Subscriber('/{}/my_pose'.format(self.name), MyPose, self.my_pose_callback)
        self.my_pose = MyPose()

        self.sub_enemy_pose_from_camera = rospy.Subscriber('/{}/enemy_pose_from_camera'.format(self.name), MyPose, self.enemy_pose_from_camera_callback)
        self.enemy_pose_from_camera = MyPose()

        # pub
        self.pub_strategy = rospy.Publisher('/{}/strategy'.format(self.name), String, queue_size=1)
        self.pub_state_stop = rospy.Publisher('/{}/state_stop'.format(self.name), Bool, queue_size=1)


    def score_callback(self, data):
        self.score = data.data

    def my_pose_callback(self,data):
        self.my_pose = data

    def enemy_pos_from_lider_callback(self,data):
        self.enemy_pos_from_lider["enemy_pos"]=data
        self.enemy_pos_from_lider["is_topic_receive"]=True

    def enemy_pose_from_camera_callback(self, data):
        self.enemy_pose_from_camera = data

    def enemy_pos_from_score_callback(self,data):
        self.enemy_pos_from_score=data
    
    def rem_time_callback(self, data):
        self.rem_time = data

    def calc_both_points(self):
        my_points = 0
        enemy_points = 0
        leftover_points = 0
        print(self.score)
        for index, value in enumerate(self.score):

            if index < 12:
                if value == 1:
                    my_points = my_points+1
                elif value == -1:
                    enemy_points = enemy_points+1
                else:
                    leftover_points = leftover_points+1
            elif index < 14:
                if value == -1:
                    enemy_points = enemy_points+3
                else:
                    leftover_points = leftover_points+3
            elif index == 14:
                if value == -1:
                    enemy_points = enemy_points+5
                else:
                    leftover_points = leftover_points+5
            elif index < 17:
                if value == 1:
                    my_points =my_points+3
                else:
                    leftover_points = leftover_points+3
            elif index == 17:
                if value == 1:
                    my_points = my_points+5
                else:
                    leftover_points = leftover_points+5

        return my_points, enemy_points, leftover_points


    def calc_distance_enemy_me(self):
        _dis = 1.0
        try:
            _x = self.my_pose.pos.x - self.enemy_pos_from_lider["enemy_pos"].x
            _y = self.my_pose.pos.y - self.enemy_pos_from_lider["enemy_pos"].y
            _dis = math.sqrt(_x*_x + _y*_y)
            print('dis', _dis)
            
        except Exception as e:
            print('missing calc_distance_enemy_me: ', e)
        return _dis

    def evaluate_war_situation(self):
        result = self.all_state_list[0]
        # first priority
        my_points, enemy_points, leftover_points = self.calc_both_points()
        # print('my_points, enemy_points, leftover_points', my_points, enemy_points, leftover_points)
        # second 
        


        # decide state
        if my_points - enemy_points > 5:
            result = self.all_state_list[1]
        elif enemy_points - my_points > 5:
            result = self.all_state_list[0]

        if self.calc_distance_enemy_me() < 0.8:
            result = self.all_state_list[1]

        return result




    def strategy_run(self):
        r=rospy.Rate(5)
        state = ''
        prev_state = ''
        preserve_count = 0
        while not rospy.is_shutdown():
            prev_state = state
            state = self.evaluate_war_situation()

            if preserve_count > 0:
                preserve_count = preserve_count-1
                print('preserve_count', preserve_count)
                if preserve_count == 0:
                    self.pub_state_stop.publish(True)
            if state != prev_state and preserve_count <= 0:
                if state == self.all_state_list[1]:
                    preserve_count = 70
                self.pub_state_stop.publish(True)
            self.pub_strategy.publish(state)
            r.sleep()
        


if __name__ == "__main__":
    rospy.init_node("BDA_strategy")
    bda_strategy = BDA_strategy()
    bda_strategy.strategy_run()
