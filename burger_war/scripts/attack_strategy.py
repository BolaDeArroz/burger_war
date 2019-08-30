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

    def init_vars(self, move, size, width, forced, t):
        self.detection_data = True
        self.move_data = move
        self.size_data = size
        self.width_data = width
        self.forced = forced
        self.last_my_map = get_my_map(self.tf_listener, self.name)
        self.last_enemy_local = calc_enemy_local(move, size, width)
        self.last_time = t
        self.detection_time = 0

    def run(self, move, size, width, forced):
        self.init_vars(move, size, width, forced, rospy.Time.now())

        while not rospy.is_shutdown():
            res = self.strategy(rospy.Time.now())

            if res != ATK_STRATEGY_CONTINUE:
                break

            self.rate.sleep()

        if res == ATK_STRATEGY_ATTACK:
            enemy_map = local_to_map(self.last_enemy_local, self.last_my_map)

            AttackBot().attack_war(enemy_map.p[0], enemy_map.p[1], enemy_map.a)

        return res

    def strategy(self, t):
        dt = (t - self.last_time).to_sec()

        self.last_my_map = get_my_map(self.tf_listener, self.name)

        if (dt < 0.001) or (self.last_my_map is None):
            return ATK_STRATEGY_CONTINUE

        enemy_local = calc_enemy_local(self.move_data, self.size_data, self.width_data)

        vx, vy = calc_velocity(enemy_local.carte, self.last_enemy_local.carte, dt)

        self.last_enemy_local = enemy_local
        self.last_time = t
        self.detection_time += dt
        print("[AttackStrategy]", self.last_my_map.p, self.last_enemy_local.polar)

        if (self.forced):
            print("[AttackStrategy] Attack (Forced)")
            return ATK_STRATEGY_ATTACK

        elif (self.last_enemy_local.polar[0] > 2.0):
            print("[AttackStrategy] Retire (Distance)", self.last_enemy_local.polar[0])
            return ATK_STRATEGY_RETIRE_DISTANCE

        #elif (not self.detection_data) or (vx * vx + vy * vy > 0.04):
        elif (not self.detection_data):
            print("[AttackStrategy] Retire (Other)", self.detection_data, vx * vx + vy * vy)
            return ATK_STRATEGY_RETIRE_OTHER

        elif (self.detection_time > 1.0) or (self.last_enemy_local.polar[0] < 0.6):
            print("[AttackStrategy] Attack", self.detection_time, self.last_enemy_local.polar[0])
            return ATK_STRATEGY_ATTACK

        else:
            print("[AttackStrategy] Continue")
            return ATK_STRATEGY_CONTINUE

    def move_callback(self, data):
        self.detection_data = int(data.data[0]) != 0
        self.move_data = int(data.data[1])
        self.size_data = int(data.data[2])
        self.width_data = int(data.data[3])


class LocalPoint():
    def __init__(self, x, y, ds, th):
        self.carte = x, y
        self.polar = ds, th


class MapInfo():
    def __init__(self, x, y, a):
        self.p = x, y
        self.a = a


def get_my_map(tf_listener, bot_name):
    try:
        ts, qt = tf_listener.lookupTransform(bot_name + "/map", bot_name + "/base_link", rospy.Time())

        el = tf.transformations.euler_from_quaternion(qt)

        return MapInfo(ts[0], ts[1], el[2])

    except Exception as e:
        print("[AttackStrategy] Exception", e)
        return None


def calc_enemy_local(move, size, width):
    r = float(size) / 2

    area = r * r * math.pi

    ds = (-1.1492 * area + 1891.2) / 1000
    th = (math.pi / 2) - (math.pi / 6) * (float(move) / width * 2)

    x = ds * math.cos(th)
    y = ds * math.sin(th)

    return LocalPoint(x, y, ds, th)


def calc_velocity(old, new, dt):
    dx, dy = new[0] - old[0], new[1] - old[1]

    return dx / dt, dy / dt


def local_to_map(local, my_map):
    a = -my_map.a + math.pi / 2

    mx = local.carte[0] * math.cos(a) + local.carte[1] * math.sin(a) + my_map.p[0]
    my = local.carte[1] * math.cos(a) - local.carte[0] * math.sin(a) + my_map.p[1]
    ma = my_map.a - local.polar[1]

    return MapInfo(mx, my, ma)


ATK_STRATEGY_CONTINUE = 0x00
ATK_STRATEGY_ATTACK = 0x01
ATK_STRATEGY_RETIRE_DISTANCE = 0x02
ATK_STRATEGY_RETIRE_OTHER = 0x03
