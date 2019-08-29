#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import tf

from std_msgs.msg import Float32MultiArray

from attackrun import AttackBot


class AttackStrategy():
    def __init__(self):
        self.name = rospy.get_param("~robot_name")
        self.rate = rospy.Rate(30)
        self.tf_listener = tf.TransformListener()
        self.mov_sub = rospy.Subscriber("array", Float32MultiArray, self.move_callback)

    def init_vars(self, move, size, width, t):
        x, y, _ = calc_enemy_local(move, size, width)

        self.detection_data = True
        self.move_data = move
        self.size_data = size
        self.width_data = width
        self.last_my_map = None
        self.last_enemy_local = (x, y)
        self.last_time = t
        self.detection_time = 0

    def run(self, move, size, width):
        self.init_vars(move, size, width, rospy.Time.now())

        while not rospy.is_shutdown():
            res = self.strategy(rospy.Time.now())

            if res != ATK_STRATEGY_CONTINUE:
                break

            self.rate.sleep()

        if res == ATK_STRATEGY_ATTACK:
            mx, my, ma = local_to_map(self.last_enemy_local, self.last_my_map)

            AttackBot().attack_war(mx, my, ma)

    def strategy(self, t):
        dt = (t - self.last_time).to_sec()

        self.last_my_map = get_my_map(self.tf_listener, self.name)

        if (dt < 0.001) or (self.last_my_map is None):
            return ATK_STRATEGY_CONTINUE

        x, y, ds = calc_enemy_local(self.move_data, self.size_data, self.width_data)

        vx, vy = calc_velocity((x, y), self.last_enemy_local, dt)

        self.last_enemy_local = (x, y)
        self.last_time = t
        self.detection_time += dt

        if (not self.detection_data) or (ds > 1.7) or (vx * vx + vy * vy > 0.04):
            print("[AttackStrategy] Retire", self.detection_data, ds, vx * vx + vy * vy)
            return ATK_STRATEGY_RETIRE

        elif (self.detection_time > 1.0) or (ds < 0.6):
            print("[AttackStrategy] Attack", self.detection_time, ds)
            return ATK_STRATEGY_ATTACK

        else:
            print("[AttackStrategy] Continue")
            return ATK_STRATEGY_CONTINUE

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
    area = float(size) / 2 * math.pi

    ds = (-1.1492 * area + 1891.2) / 1000
    th = (math.pi / 2) - (math.pi / 6) * (float(move) / width * 2)

    x = ds * math.cos(th)
    y = ds * math.sin(th)

    return x, y, ds


def calc_velocity(old, new, dt):
    dx, dy = new[0] - old[0], new[1] - old[1]

    return dx / dt, dy / dt


def local_to_map(local, my_map):
    ma = -my_map[2] + math.pi / 2

    mx = local[0] * math.cos(ma) + local[1] * math.sin(ma) + my_map[0]
    my = local[1] * math.cos(ma) - local[0] * math.sin(ma) + my_map[1]

    return mx, my, ma


ATK_STRATEGY_CONTINUE = 0x00
ATK_STRATEGY_ATTACK = 0x01
ATK_STRATEGY_RETIRE = 0x02
