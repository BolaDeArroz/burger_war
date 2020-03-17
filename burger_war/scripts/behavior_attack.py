#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
import actionlib_msgs
import math
import rospy
import smach
import smach_ros
import tf

from geometry_msgs.msg import Point, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, Float32MultiArray, Int32MultiArray

import my_move_base

from burger_war.msg import MyPose


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

        self.score = None

        self.enemy_pos = EnemyPos()

        self.my_pose = None

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        rospy.Subscriber(
                'state_stop', Bool, self.stop_callback)
        rospy.Subscriber(
                'score', Int32MultiArray, self.score_callback)
        rospy.Subscriber(
                'my_pose', MyPose, self.my_pose_callback)

    def reset(self):
        self.is_stop_receive = False

        self.client.cancel_goal()

        self.pub_vel()

    def is_data_exists(self):
        return ((self.score is not None) and
                (self.enemy_pos.is_data_exists()) and
                (self.my_pose is not None))

    def stop_callback(self, data):
        if data.data:
            self.is_stop_receive = True

    def score_callback(self, data):
        self.score = data.data

    def my_pose_callback(self,data):
        self.my_pose = data

    def check_stop(self):
        return self.is_stop_receive

    def check_score(self):
        return [i for i, e in enumerate(self.score) if e > 0]

    def check_enemy_pos(self):
        return self.enemy_pos.check()

    def check_my_pose(self):
        return self.my_pose

    def check_client(self):
        return self.client.get_state()

    def set_goal(self, x, y, yaw):
        my_move_base.setGoal(self.client, x, y, yaw)

    def pub_vel(self, x=0, y=0, a=0):
        msg = Twist()

        msg.linear.x = x
        msg.linear.y = y

        msg.angular.z = a

        self.cmd_vel.publish(msg)


class EnemyPos:
    def __init__(self):
        self.enemy_pos_from = None

        self.enemy_pos_from_score = None
        self.enemy_pos_from_lider = None

        rospy.Subscriber(
                'enemy_pos_from_score', Float32MultiArray, self.score_callback)
        rospy.Subscriber(
                'enemy_pos_from_lider', Point, self.lider_callback)

    def is_data_exists(self):
        return self.enemy_pos_from is not None

    def score_callback(self, data):
        if ((self.enemy_pos_from_score is None) or
            (max(data.data) > max(self.enemy_pos_from_score))):
            self.enemy_pos_from = 'score'

        self.enemy_pos_from_score = data.data

    def lider_callback(self, data):
        self.enemy_pos_from = 'lider'

        self.enemy_pos_from_lider = data

    def check(self):
        x, y = 0, 0

        if self.enemy_pos_from == 'score':
            for i in range(len(ENEMY_POS_MAP)):
                x += self.enemy_pos_from_score[i] * ENEMY_POS_MAP[i][0]
                y += self.enemy_pos_from_score[i] * ENEMY_POS_MAP[i][1]

        if self.enemy_pos_from == 'lider':
            x = self.enemy_pos_from_lider.x
            y = self.enemy_pos_from_lider.y

        return x, y


class Selecting(smach.State):
    def __init__(self, func):
        outcomes = ['success', 'end']

        keys = ['target']

        smach.State.__init__(
                self, outcomes=outcomes, input_keys=keys, output_keys=keys)

        self.func = func

        self.is_first = True

    def execute(self, userdata):
        self.func.reset()

        userdata.target = None

        while not rospy.is_shutdown():
            if not self.func.is_data_exists():
                continue

            self.select(userdata)

            result = self.check(userdata)

            if result is not None:
                self.func.reset()

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
        if self.is_first:
            self.select_first(userdata)

            self.is_first = False

        else:
            self.select_nth(userdata)

    def select_first(self, userdata):
        userdata.target = TARGET_FIRST

    def select_nth(self, userdata):
        mypos = self.func.check_my_pose()
        enemy = self.func.check_enemy_pos()

        costs = [x.bcost for x in MK_INFOS]

        for i in self.func.check_score():
            if i < len(MK_INFOS):
                costs[i] += K_MY_MARKER

        for i in range(len(costs)):
            p = MK_INFOS[i].point

            costs[i] += self.distance(p, mypos.pos.x, mypos.pos.y) * K_MY_POSE

            costs[i] -= self.distance(p, enemy[0], enemy[1]) * K_ENEMY_POS

        cost = min(costs)

        if cost < K_MY_MARKER:
            userdata.target = costs.index(cost)

    def distance(self, point, x, y):
        dx = point[0] - x
        dy = point[1] - y

        return math.sqrt(dx * dx + dy * dy)


class Moving(smach.State):
    def __init__(self, func):
        outcomes = ['success', 'fail', 'read', 'end']

        keys = ['target']

        smach.State.__init__(
                self, outcomes=outcomes, input_keys=keys, output_keys=keys)

        self.func = func

    def execute(self, userdata):
        self.func.reset()
        self.func.set_goal(*(MK_INFOS[userdata.target].point))

        while not rospy.is_shutdown():
            if not self.func.is_data_exists():
                continue

            result = self.check(userdata)

            if result is not None:
                self.func.reset()

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
        outcomes = ['success', 'timeout', 'end']

        keys = ['target']

        smach.State.__init__(
                self, outcomes=outcomes, input_keys=keys, output_keys=keys)

        self.func = func

        self.start = rospy.Time.now()

    def execute(self, userdata):
        self.func.reset()
        self.func.pub_vel(*VEL_SPIN)

        self.start = rospy.Time.now()

        while not rospy.is_shutdown():
            if not self.func.is_data_exists():
                continue

            result = self.check(userdata)

            if result is not None:
                self.func.reset()

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


class MkInfo:
    def __init__(self, point, bcost):
        self.point = point
        self.bcost = bcost


RATE = 10


TIMEOUT_READING = 5


VEL_SPIN = (0, 0, math.pi / 2)


TARGET_FIRST = 10


K_MY_MARKER = 10000

K_MY_POSE = 1.0

K_ENEMY_POS = 0.5


MK_INFOS = eval("""
[
        MkInfo((-0.530,  0.800, -math.pi / 2), 0.01),
        MkInfo(( 0.530,  0.800, -math.pi / 2), 0.01),
        MkInfo((-0.530,  0.260,  math.pi / 2), 0.03),
        MkInfo(( 0.530,  0.260,  math.pi / 2), 0.03),
        MkInfo((     0,  0.340, -math.pi / 2), 0.05),
        MkInfo((-0.340,      0,  0),           0.05),
        MkInfo(( 0.340,      0,  math.pi),     0.05),
        MkInfo((     0, -0.340,  math.pi / 2), 0.05),
        MkInfo((-0.530, -0.260, -math.pi / 2), 0.03),
        MkInfo(( 0.530, -0.260, -math.pi / 2), 0.03),
        MkInfo((-0.530, -0.800,  math.pi / 2), 0.01),
        MkInfo(( 0.530, -0.800,  math.pi / 2), 0.01)
]
""")


ENEMY_POS_MAP = eval("""
[
        (-0.265,  1.325),
        ( 0.265,  1.325),
        (-0.795,  0.795),
        (-0.265,  0.795),
        ( 0.265,  0.795),
        ( 0.795,  0.795),
        (-1.325,  0.265),
        (-0.795,  0.265),
        (-0.265,  0.265),
        ( 0.265,  0.265),
        ( 0.795,  0.265),
        ( 1.325,  0.265),
        (-1.325, -0.265),
        (-0.795, -0.265),
        (-0.265, -0.265),
        ( 0.265, -0.265),
        ( 0.795, -0.265),
        ( 1.325, -0.265),
        (-0.795, -0.795),
        (-0.265, -0.795),
        ( 0.265, -0.795),
        ( 0.795, -0.795),
        (-0.265, -1.325),
        ( 0.265, -1.325)
]
""")
