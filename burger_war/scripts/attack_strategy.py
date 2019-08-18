#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import math
import numpy as np
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class AttackStrategy():
    def __init__(self):
        rospy.init_node("attack_strategy")

        self.is_detected = False
        self.scan_data = LaserScan()
        self.detection_data = None
        self.last_position = (0, 0)
        self.last_time = rospy.Time.now()
        self.detection_time = 0

        self.atk_pub = rospy.Publisher("atk_request", Bool, queue_size=1)
        self.run_pub = rospy.Publisher("run_request", Bool, queue_size=1)
        self.det_sub = rospy.Subscriber("move_x", Bool, self.detection_callback)
        self.scn_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback)

    def run(self):
        r = rospy.Rate(200)

        while not rospy.is_shutdown():
            self.strategy()

            r.sleep()

    def strategy(self):
        if (not self.is_detected) and (self.detection_data is not None):
            self.start_judge()

        elif self.is_detected and (self.detection_data is not None):
            self.continue_judge()

        elif self.is_detected and (self.detection_data is None):
            self.retire()

    def start_judge(self):
        _, x, y = calc_enemy_position(1000, 0) # From move_x

        self.is_detected = True
        self.last_position = (x, y)
        self.last_time = rospy.Time.now()
        self.detection_time = 0

    def continue_judge(self):
        t = rospy.Time.now()

        dt = (t - self.last_time).to_sec()

        if dt < 0.1:
            return

        d, x, y = calc_enemy_position(1000, 0) # From move_x

        dx, dy = x - self.last_position[0], y - self.last_position[1]

        vx, vy = dx / dt, dy / dt

        self.last_position = (x, y)
        self.last_time = t
        self.detection_time += dt

        if (d > 1500) or (vx * vx + vy * vy > 100 * 100):
            self.retire()

        elif (self.detection_time > 2.0) or (d < 500):
            self.attack()

    def attack(self):
        print("[AttackStrategy] Attack")
        self.atk_pub.publish(True)

    def retire(self):
        print("[AttackStrategy] Retire")
        self.is_detected = False

        self.run_pub.publish(True)

    def detection_callback(self, data):
        self.detection_data = data

    def scan_callback(self, data):
        self.scan_data = data


def calc_enemy_position(size, diff):
    d = -1.1492 * size + 1891.2
    t = 0 # From diff (angle)

    x = d * math.cos(t)
    y = d * math.sin(t)

    return d, x, y


if __name__ == "__main__":
    AttackStrategy().run()
