#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import math
import numpy as np
import rospy
import tf

from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray

import attackrun as atk


class AttackStrategy():
    def __init__(self):
        self.name = "red_bot"
        self.tf_listener = tf.TransformListener()
        self.detection_data = True
        self.scan_data = LaserScan()
        self.my_map = None
        self.last_time = rospy.Time.now()
        self.detection_time = 0

        self.mov_sub = rospy.Subscriber("array", Float32MultiArray, self.move_callback)

    def run(self, move, size):
        print("[Strategy]")
        r = rospy.Rate(30)

        self.move_data = move
        self.size_data = size

        x, y, _, th = calc_enemy_local(move, size, 0)

        self.last_local = (x, y, th)

        while not rospy.is_shutdown():
            res = self.strategy()

            if res != 0x00:
                #print("[Retire]")
                break

            r.sleep()

        if res == 0x01:
            print("[Attack]")            
            mx, my = local_to_map(self.last_local[0], self.last_local[1], self.my_map[0], self.my_map[1], self.my_map[2])

            atk.AttackBot().attack_war(mx, my, self.last_local[2])


    def strategy(self):
        t = rospy.Time.now()

        dt = (t - self.last_time).to_sec()

        self.my_map = get_my_map(self.tf_listener, self.name)

        if (dt < 0.001) or (self.my_map is None):
            print("[Through]", dt)
            return 0x00

        x, y, ds, th = calc_enemy_local(self.move_data, self.size_data, 0)

        dx, dy = x - self.last_local[0], y - self.last_local[1]

        vx, vy = dx / dt, dy / dt

        self.last_local = (x, y, th)
        self.last_time = t
        self.detection_time += dt

        if (not self.detection_data) or (ds > 1.5) or (vx * vx + vy * vy > 0.5 * 0.5):
            return 0x02

        elif (self.detection_time > 2.0) or (ds < 0.5):
            return 0x01

    def move_callback(self, data):
        self.detection_data = int(data.data[0]) != 0
        self.move_data = int(data.data[1])
        self.size_data = int(data.data[2])
        self.width_data = int(data.data[3])
        self.height_data = int(data.data[4])

    def scan_callback(self, data):
        self.scan_data = data


def get_my_map(tf_listener, name):
    try:
        trans, quaternion = tf_listener.lookupTransform(name +"/map", name +"/base_link", rospy.Time())

        euler = tf.transformations.euler_from_quaternion(quaternion)

        return trans[0], trans[1], euler[2]

    except Exception as e:
        print(e)
        return None


def calc_enemy_local(move, size, width):
    ds = (-1.1492 * size + 1891.2) / 1000
    th = (math.pi / 2) - (math.pi / 3) * (move / 400.0)

    x = ds * math.cos(th)
    y = ds * math.sin(th)

    return x, y, ds, th


def local_to_map(x, y, ox, oy, a):
    mx = x * math.cos(a) - y * math.sin(a) + ox
    my = x * math.sin(a) + y * math.cos(a) + oy

    return mx, my


if __name__ == "__main__":
    AttackStrategy().run(0, 0)
