#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import json
import rospy


from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String



class PubScore():
    def __init__(self):
        # bot name 
        robot_name=''
        self.name = robot_name
        # markers name: order is decided by zyali's board
        self.markers_name_list = [
            'Tomato_N', 'Omelette_N',
            'Tomato_S', 'Omelette_S', 
            'FriedShrimp_N','FriedShrimp_W', 
            'FriedShrimp_E','FriedShrimp_S', 
            'Pudding_N','OctopusWiener_N', 
            'Pudding_S', 'OctopusWiener_S', 
            'RE_L', 'RE_R', 'RE_B',
            'BL_L', 'BL_R', 'BL_B']

        # sub
        self.war_state=None
        self.war_state_sub = rospy.Subscriber('/{}/war_state'.format(self.name), String, self.warstate_callback)
        # pub
        self.score_pub = rospy.Publisher("score", Int32MultiArray, queue_size=1)

    def warstate_callback(self, data):
        _array = Int32MultiArray()
        self.war_state = data

        result = self.check_marker(self.war_state)
        _array.data = result
        # print("pub score ", result)
        self.score_pub.publish(_array)
        

    def check_marker(self, war_state):
        """
        
        """
        red_list = []
        blue_list = []
        targets_list = []
        result_list = []
        if war_state is None:
            return red_list
        json_war_state = json.loads(war_state.data)
        for key in json_war_state:
            if key == 'targets':
                targets_list = json_war_state[key]
                break
        for target in targets_list:
            for key in target:
                if key == 'player' and target[key] == 'r':
                    red_list.append(target['name'])
                elif key == 'player' and target[key] == 'b':
                    blue_list.append(target['name'])
        for index, name in enumerate(self.markers_name_list):
            if name in red_list:
                result_list.append(1)
            elif name in blue_list:
                result_list.append(-1)
            else:
                result_list.append(0)
        return result_list
        


if __name__ == "__main__":
    rospy.init_node("pub_score")
    pub_score = PubScore()
    rospy.spin()
