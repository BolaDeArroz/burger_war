#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import math
import numpy as np
import rospy
import tf

from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

from attackrun import AttackBot


class AttackStrategy():
    def __init__(self):
        self.name = rospy.get_param("~robot_name")
        self.rate = rospy.Rate(30)
        self.tf_listener = tf.TransformListener()
        self.mov_sub = rospy.Subscriber("array", Float32MultiArray, self.move_callback)

    def init_vars(self, move, size, width, t):
        x, y, _, th = calc_enemy_local(move, size, width)

        self.detection_data = True
        self.move_data = move
        self.size_data = size
        self.width_data = width
        self.my_map = None
        self.last_local = (x, y, th)
        self.last_time = t
        self.detection_time = 0

    def run(self, move, size, width):
        print("[AttackStrategy] Start")

        self.init_vars(move, size, width, rospy.Time.now())

        while not rospy.is_shutdown():
            res = self.strategy(rospy.Time.now())

            if res != 0x00:
                break

            self.rate.sleep()

        if res == 0x01:            
            mx, my = local_to_map(self.last_local[0], self.last_local[1], self.my_map[0], self.my_map[1], self.my_map[2])

            AttackBot().attack_war(mx, my, self.last_local[2])

    def strategy(self, t):
        dt = (t - self.last_time).to_sec()

        self.my_map = get_my_map(self.tf_listener, self.name)

        if (dt < 0.001) or (self.my_map is None):
            return 0x00

        x, y, ds, th = calc_enemy_local(self.move_data, self.size_data, self.width_data)

        dx, dy = x - self.last_local[0], y - self.last_local[1]
        vx, vy = dx / dt, dy / dt

        self.last_local = (x, y, th)
        self.last_time = t
        self.detection_time += dt

        if (not self.detection_data) or (ds > 2.0) or (vx * vx + vy * vy > 0.25 * 0.25):
            print("[AttackStrategy] Retire", self.detection_data, ds, vx * vx + vy * vy)
            return 0x02

        elif (self.detection_time > 2.0) or (ds < 0.5):
            print("[AttackStrategy] Attack", self.detection_time, ds)
            return 0x01

        else:
            print("[AttackStrategy] Continue")
            return 0x00

    def move_callback(self, data):
        self.detection_data = int(data.data[0]) != 0
        self.move_data = int(data.data[1])
        self.size_data = int(data.data[2])
        self.width_data = int(data.data[3])


def get_my_map(tf_listener, bot_name):
    try:
        trans, quaternion = tf_listener.lookupTransform(bot_name + "/map", bot_name + "/base_link", rospy.Time())

        euler = tf.transformations.euler_from_quaternion(quaternion)

        return trans[0], trans[1], euler[2]

    except Exception as e:
        print("[AttackStrategy] Exception", e)
        return None


def calc_enemy_local(move, size, width):
    area = size / 2 * math.pi

    ds = (-1.1492 * area + 1891.2) / 1000
    th = (math.pi / 2) - (math.pi / 3) * (move / width * 2)

    x = ds * math.cos(th)
    y = ds * math.sin(th)

    return x, y, ds, th


def local_to_map(x, y, ox, oy, a):
    r = a + math.pi / 2

    mx = x * math.cos(r) - y * math.sin(r) + ox
    my = x * math.sin(r) + y * math.cos(r) + oy

    return mx, my


if __name__ == "__main__":
    AttackStrategy().run(0, 0)
