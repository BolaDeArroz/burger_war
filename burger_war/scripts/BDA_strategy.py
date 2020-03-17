#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import json
import rospy
import math

import threading
import smach_ros

from smach_msgs.msg import SmachContainerStatus,SmachContainerInitialStatusCmd,SmachContainerStructure
from std_msgs.msg import Float32MultiArray, Time, String, Bool, Int32MultiArray
from geometry_msgs.msg import Point
from burger_war.msg import MyPose 




class BDA_strategy():
    def __init__(self):
        # bot name 
        robot_name=''
        self.name = robot_name
        self.all_state_list = ['attack', 'escape', 'disturb']

        # Message subscribers
        self._structure_subs = {}
        # smach introspection client
        self._client = smach_ros.IntrospectionClient()
        self._containers= {}
        self._selected_paths = []

        # Message subscribers
        self._structure_subs = {}
        self._status_subs = {}

        # before camera result
        self._prev_camera_result = [0.0, 0.0]

        # sub
        """
        use almost all the our publish data
        - enemy_pos_from_score
        - pub_score
        - enemy_pos_from_lider
        - rem_time
        - my_pose
        - enemy_pose_from_camera
        present status
        - 
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

        self.current_state = None

        # pub
        self.pub_strategy = rospy.Publisher('/{}/strategy'.format(self.name), String, queue_size=1)
        self.pub_state_stop = rospy.Publisher('/{}/state_stop'.format(self.name), Bool, queue_size=1)

        # thread
        # Start a thread in the background to update the server list
        self._keep_running = True
        self._server_list_thread = threading.Thread(target=self._update_server_list)
        self._server_list_thread.start()
        # Start a thread in the background to check stagnation
        self._stagnation = False
        self._check_stagnation_thread = threading.Thread(target=self._check_stagnation)
        self._check_stagnation_thread.start()


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
        

    def state_callback(self, data):
        """
        Process status messages.
        """
        if data.path == '/SM_ROOT':
            self.current_state = data.active_states[0]
            # print('current state', self.current_state[0])


    def _check_stagnation(self):
        stag_count = 0
        prev_stag_count = 0
        x_prev_my_pos = self.my_pose.pos.x
        y_prev_my_pos = self.my_pose.pos.y
        while self._keep_running:
            self._stagnation = False
            
            if abs(x_prev_my_pos - self.my_pose.pos.x) < 0.01:
                if abs(y_prev_my_pos - self.my_pose.pos.y) < 0.01:
                    stag_count = stag_count+1
                    # 3 straight count
                    if stag_count > 3:
                        print('confirm stagnation!!')
                        self._stagnation = True
                        stag_count = 0
            if prev_stag_count == stag_count:
                stag_count = 0
            prev_stag_count = stag_count


            x_prev_my_pos = self.my_pose.pos.x
            y_prev_my_pos = self.my_pose.pos.y
            print('x, y: ', x_prev_my_pos, y_prev_my_pos)
            print('stag_count: ', stag_count)
            
            rospy.sleep(1.0)


    def _update_server_list(self):
        """
        Update the list of known SMACH introspection servers
        """
        while self._keep_running:
            # Update the server list
            server_names = self._client.get_servers()
            new_server_names = [sn for sn in server_names if sn not in self._status_subs]

            # Create subscribers for new servers
            for server_name in new_server_names:
                self._status_subs[server_name] = rospy.Subscriber(
                        server_name+smach_ros.introspection.STATUS_TOPIC,
                        SmachContainerStatus,
                        callback = self.state_callback,
                        queue_size=50)
            # This doesn't need to happen very often
            rospy.sleep(1.0)
        

    def calc_both_points(self):
        my_points = 0
        enemy_points = 0
        leftover_points = 0
        cheese_points = False
        # print(self.score)
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
                    leftover_points = leftover_points+3
            elif index == 14:
                if value == -1:
                    enemy_points = enemy_points+5
                    leftover_points = leftover_points+5
            elif index < 17:
                if value == 1:
                    my_points =my_points+3
            elif index == 17:
                if value == 1:
                    my_points = my_points+5
            
            # for cheese
            if index == 4 and value == -1:
                cheese_points = True

        return my_points, enemy_points, leftover_points, cheese_points


    def calc_distance_enemy_me(self):
        _dis = 1.0
        try:
            _x = self.my_pose.pos.x - self.enemy_pos_from_lider["enemy_pos"].x
            _y = self.my_pose.pos.y - self.enemy_pos_from_lider["enemy_pos"].y
            _dis = math.sqrt(_x*_x + _y*_y)
            
        except Exception as e:
            print('missing calc_distance_enemy_me: ', e)
        return _dis


    def check_enemy_area(self):
        result = False
        _dis = 1.0
        try:
            _enemy_pos_x = self.enemy_pose_from_camera.pos.x
            _enemy_pos_y = self.enemy_pose_from_camera.pos.y

            if (self._prev_camera_result[0] != _enemy_pos_x) and (self._prev_camera_result[1] != _enemy_pos_y):
                # update
                self._prev_camera_result[0] = _enemy_pos_x
                self._prev_camera_result[1] = _enemy_pos_y

                _x = self.my_pose.pos.x - self.enemy_pose_from_camera.pos.x
                _y = self.my_pose.pos.y - self.enemy_pose_from_camera.pos.y
                _dis = math.sqrt(_x*_x + _y*_y)
                # print('dis', _dis)
                if _dis < 0.5:
                    return True
                else:
                    return False

        except Exception as e:
            return False
            

    def evaluate_war_situation(self):
        """
        decide state
        """
        result = self.all_state_list[0]
        # first priority
        my_points, enemy_points, leftover_points, cheese_points = self.calc_both_points()
        
        # camera
        if self.check_enemy_area():
            result = self.all_state_list[1]
        
        # my points 
        if enemy_points - my_points > 5:
            result = self.all_state_list[0]

        
        if self.calc_distance_enemy_me() < 0.7:
            result = self.all_state_list[1]

        if my_points - enemy_points > 9:
            result = self.all_state_list[0]

        # for cheese burger
        if cheese_points and enemy_points<=1 and my_points>7 and self.rem_time.data.to_sec()>20:
            result = self.all_state_list[2]
        if cheese_points and enemy_points<=1 and self.rem_time.data.to_sec()<80:
            result = self.all_state_list[0]


        # lost my marker points
        if leftover_points >= 11:
            result = self.all_state_list[0]
        # check stagnation
        if self._stagnation:
            result = self.all_state_list[2]
            print('****************** stagnation ***************************')
        
        
        return result


    def strategy_run(self):
        r=rospy.Rate(5)
        state = ''
        prev_state = ''
        prev_real_state = ''
        stop_send_result = False
        preserve_count = 0
        resend_count = 0
        while not rospy.is_shutdown():
            prev_state = state
            print('state_count : ', preserve_count)
            state = self.evaluate_war_situation()

            # change state
            if preserve_count > 0:
                preserve_count = preserve_count-1
                # print('preserve_count', preserve_count)
                if preserve_count == 0:
                    self.pub_state_stop.publish(True)
                    stop_send_result = True
            # stop topic
            if state != prev_state and preserve_count <= 0:
                # change to attack
                if state == self.all_state_list[0]:
                    preserve_count = 0
                # change to escape
                elif state == self.all_state_list[1]:
                    preserve_count = 35
                # change to disturb
                elif state == self.all_state_list[2]:
                    preserve_count = 10
                else:
                    preserve_count = 0
                stop_send_result = True

                # self.pub_state_stop.publish(True)
                # stop_send_result = True
            # check chaned result
            if stop_send_result == True:
                # resend
                if prev_real_state == self.current_state and resend_count<10:
                    self.pub_state_stop.publish(True)
                    print('++++++++++++ resend +++++++++++++')
                    resend_count = resend_count+1
                else:
                    stop_send_result = False
                    prev_real_state = self.current_state
                    resend_count = 0
            
            # change state topic
            self.pub_strategy.publish(state)
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("BDA_strategy")
    bda_strategy = BDA_strategy()
    bda_strategy.strategy_run()



