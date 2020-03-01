#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
import actionlib_msgs
import math
import rospy
import smach
import smach_ros
import tf

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, Float32MultiArray, Int32MultiArray

import my_move_base

class behavior_attack(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome'])

        self.sm_sub = smach.StateMachine(outcomes=['outcome'])

        with self.sm_sub:
            func = CommonFunction()

            smach.StateMachine.add('Selecting', Selecting(func), transitions={
                    'success': 'Moving',
                    'end': 'outcome'
            })
            smach.StateMachine.add('Moving', Moving(func), transitions={
                    'success': 'Reading',
                    'fail': 'Selecting',
                    'read': 'Selecting',
                    'end': 'outcome'
            })
            smach.StateMachine.add('Reading', Reading(func), transitions={
                    'success': 'Selecting',
                    'timeout': 'Selecting',
                    'end': 'outcome'
            })

        sis = smach_ros.IntrospectionServer(
                'server_name', self.sm_sub, '/SM_ATTACK')

        sis.start()

    def execute(self, userdata):
        self.sm_sub.execute()

        return 'outcome'


class CommonFunction:
    def __init__(self):
        self.is_stop_receive = False

        self.score = []

        self.enemy_pos_from_score = []

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.Subscriber(
                'state_stop', Bool, self.stop_callback)
        rospy.Subscriber(
                'pub_score', Int32MultiArray, self.score_callback)
        rospy.Subscriber(
                'enemy_pos_from_score', Float32MultiArray, self.epfs_callback)

    def reset(self):
        self.is_stop_receive = False

    def is_data_exists(self):
        return (len(self.score) > 0 and
                len(self.enemy_pos_from_score) > 0)

    def stop_callback(self, data):
        if data.data:
            self.is_stop_receive = True

    def score_callback(self, data):
        self.score = data.data

    def epfs_callback(self, data):
        self.enemy_pos_from_score = data.data

    def check_stop(self):
        result = self.is_stop_receive

        self.is_stop_receive = False

        return result

    def check_score(self):
        result = [i for i, e in enumerate(self.score) if e > 0]

        return result

    def check_enemy_pos_from_score(self):
        return self.enemy_pos_from_score

    def check_client(self):
        return self.client.get_state()

    def set_goal(self, x, y, yaw):
        my_move_base.setGoal(self.client, x / 1000.0, y / 1000.0, yaw)

    def cancel_goal(self):
        self.client.cancel_goal()


class Selecting(smach.State):
    def __init__(self, func):
        smach.State.__init__(
                self,
                outcomes=['success', 'end'],
                output_keys=['target'])

        self.func = func

    def execute(self, userdata):
        self.func.reset()

        userdata.target = None

        while not rospy.is_shutdown():
            if not self.func.is_data_exists():
                continue

            self.select(userdata)

            result = self.check(userdata)

            if result is not None:
                return result

            rospy.Rate(RATE).sleep()

        return 'end'

    def check(self, userdata):
        if self.func.check_stop():
            return 'end'

        if userdata.target is not None:
            return 'success'

        return None

    def select(self, userdata):
        # TODO: 判断材料に自己位置を導入
        # TODO: できれば経路も考慮したい
        # TODO: 履歴
        costs = BASE_COSTS[:]

        enemy = self.func.check_enemy_pos_from_score()

        for i in self.func.check_score():
            costs[i] += K_MY_MARKER

        for i, _ in enumerate(costs):
            for j in NEAR_CELLS[i]:
                costs[i] += enemy[j] * K_ENEMY_POS_FROM_SCORE

        cost = min(costs)

        if cost < K_MY_MARKER:
            userdata.target = costs.index(cost)


class Moving(smach.State):
    def __init__(self, func):
        smach.State.__init__(
                self,
                outcomes=['success', 'fail', 'read', 'end'],
                input_keys=['target'],
                output_keys=['target'])

        self.func = func

    def execute(self, userdata):
        self.func.reset()
        self.func.set_goal(*(POINTS[userdata.target]))

        while not rospy.is_shutdown():
            if not self.func.is_data_exists():
                continue

            result = self.check(userdata)

            if result is not None:
                return result

            rospy.Rate(RATE).sleep()

        return 'end'

    def check(self, userdata):
        if self.func.check_stop():
            return 'end'

        if userdata.target in self.func.check_score():
            return 'read'

        state = self.func.check_client()

        if state == actionlib_msgs.msg.GoalStatus.PENDING:
            return None
        if state == actionlib_msgs.msg.GoalStatus.ACTIVE:
            return None
        if state == actionlib_msgs.msg.GoalStatus.RECALLED:
            return 'fail'
        if state == actionlib_msgs.msg.GoalStatus.REJECTED:
            return 'fail'
        if state == actionlib_msgs.msg.GoalStatus.PREEMPTED:
            return 'fail'
        if state == actionlib_msgs.msg.GoalStatus.ABORTED:
            return 'fail'
        if state == actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            return 'success'
        if state == actionlib_msgs.msg.GoalStatus.LOST:
            return 'fail'

        return 'fail'


class Reading(smach.State):
    def __init__(self, func):
        smach.State.__init__(
                self,
                outcomes=['success', 'timeout', 'end'],
                input_keys=['target'])

        self.func = func

        self.start = rospy.Time.now()

    def execute(self, userdata):
        self.func.reset()

        self.start = rospy.Time.now()

        while not rospy.is_shutdown():
            if not self.func.is_data_exists():
                continue

            result = self.check(userdata)

            if result is not None:
                return result

            rospy.Rate(RATE).sleep()

        return 'end'

    def check(self, userdata):
        if self.func.check_stop():
            return 'end'

        if (rospy.Time.now() - self.start).to_sec() > TIMEOUT_READING:
            return 'timeout'

        if userdata.target in self.func.check_score():
            return 'success'

        return None


RATE = 10


TIMEOUT_READING = 5


K_MY_MARKER = 100


K_ENEMY_POS_FROM_SCORE = 1


POINTS = eval("""
[
        (-530,  760, -math.pi / 2),
        ( 530,  760, -math.pi / 2),
        (-530,  300,  math.pi / 2),
        ( 530,  300,  math.pi / 2),
        (   0,  300, -math.pi / 2),
        (-300,    0, -math.pi),
        ( 300,    0,  math.pi),
        (   0, -300,  math.pi / 2),
        (-530, -300, -math.pi / 2),
        ( 530, -300, -math.pi / 2),
        (-530, -760,  math.pi / 2),
        ( 530, -760,  math.pi / 2)
]
""")


BASE_COSTS = eval("""
[
        0.00, 0.00,
        0.25, 0.25,
        0.50, 0.50, 0.50, 0.50,
        0.25, 0.25,
        0.00, 0.00
]
""")


NEAR_CELLS = eval("""
[
        [ 2,  3],
        [ 4,  5],
        [ 7,  8],
        [ 9, 10],
        [ 8,  9],
        [ 8, 14],
        [ 9, 15],
        [14, 15],
        [13, 14],
        [15, 16],
        [18, 19],
        [20, 21]
]
""")
