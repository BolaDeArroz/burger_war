#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import rospy

from std_msgs.msg import Float32MultiArray, String


class EnemyPosFromScore:
    def __init__(self):
        topic_ws = str(rospy.get_param("~topic_ws"))

        self.side = str(rospy.get_param("~side"))

        self.pub = rospy.Publisher(NODE_NAME, Float32MultiArray, queue_size=1)

        self.old = {}
        self.new = {}

        self.points = INIT_POINTS[self.side]

        rospy.Subscriber(topic_ws, String, self.ws_callback)

    def execute(self):
        rate = int(rospy.get_param("~rate"))

        r = rospy.Rate(rate)

        while not rospy.is_shutdown():
            self.publish()

            r.sleep()

    def publish(self):
        msg = Float32MultiArray()

        msg.data, self.points = calc_posmap(
                self.side, self.points, self.old, self.new)

        self.old = self.new

        self.pub.publish(msg)

    def ws_callback(self, data):
        self.new = json_to_targets(json.loads(data.data))


def json_to_targets(json_data):
    targets = {}

    for data in json_data["targets"]:
        targets[data["name"]] = data["player"]

    return targets


def calc_posmap(side, points, old, new):
    diff = diff_of_targets(old, new)

    points = calc_points(side, points, diff)

    posmap = points_to_posmap(points)

    return posmap, points


def diff_of_targets(old, new):
    diff = {}

    for name in old:
        if (name in new) and (old[name] != new[name]):
            diff[name] = new[name]

        else:
            diff[name] = "u"

    return diff


def calc_points(side, points, diff):
    if INV_SIDE[side] in diff.values():
        points = init_points(side, diff)

    else:
        points = open_points(points)

    return points


def points_to_posmap(points):
    s = sum(points)

    if s <= 0:
        return [0 for _ in points]

    return [x / s for x in points]


def init_points(side, diff):
    points = [0 for _ in range(CELLS)]

    names = [k for k, v in diff.items() if v == INV_SIDE[side]]

    for name in names:
        if name in POINT_MAPPING:
            for i in POINT_MAPPING[name]:
                points[i] = 1

    return points


def open_points(points):
    clone = points[:]

    for i in range(CELLS):
        d = clone[i] * K_OPEN_POINTS

        points[i] -= d * len(NEXT_CELL_MAPPING[i])

        for j in NEXT_CELL_MAPPING[i]:
            points[j] = max(points[j], clone[j] + d)

    return points


NODE_NAME = "enemy_pos_from_score"


CELLS = 24


K_OPEN_POINTS = 0.05


INV_SIDE = {
        "r": "b",
        "b": "r"
}


INIT_POINTS = {}

INIT_POINTS["r"] = [
        1, 1,
        0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0
]

INIT_POINTS["b"] = [
        0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0,
        1, 1
]


POINT_MAPPING = eval("""
{
        "FriedShrimp_N":    [ 8,  9],
        "FriedShrimp_W":    [ 8, 14],
        "FriedShrimp_E":    [ 9, 15],
        "FriedShrimp_S":    [14, 15],
        "Tomato_N":         [ 2,  3],
        "Tomato_S":         [ 7,  8],
        "Omelette_N":       [ 4,  5],
        "Omelette_S":       [ 9, 10],
        "Pudding_N":        [13, 14],
        "Pudding_S":        [18, 19],
        "OctopusWiener_N":  [15, 16],
        "OctopusWiener_S":  [20, 21]
}
""")


NEXT_CELL_MAPPING = eval("""
[
        [ 1,  3],
        [ 0,  4],
        [ 3,  7],
        [ 0,  2,  4,  8],
        [ 1,  3,  5,  9],
        [ 4, 10],
        [ 7, 12],
        [ 2,  6,  8, 13],
        [ 3,  7,  9, 14],
        [ 4,  8, 10, 15],
        [ 5,  9, 11, 16],
        [10, 17],
        [ 6, 13],
        [ 7, 12, 14, 18],
        [ 8, 13, 15, 19],
        [ 9, 14, 16, 20],
        [10, 15, 17, 21],
        [11, 16],
        [13, 19],
        [14, 18, 20, 22],
        [15, 19, 21, 23],
        [16, 20],
        [19, 23],
        [20, 22]
]
""")


if __name__ == "__main__":
    rospy.init_node(NODE_NAME)

    node = EnemyPosFromScore()

    node.execute()
