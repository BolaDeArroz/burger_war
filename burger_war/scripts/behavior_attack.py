#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
import actionlib_msgs
import math
import rospy
import smach
import smach_ros
import tf

from geometry_msgs.msg import Twist
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

        self.enemy_pos_from_score = None

        self.my_pose = None

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        rospy.Subscriber(
                'state_stop', Bool, self.stop_callback)
        rospy.Subscriber(
                'score', Int32MultiArray, self.score_callback)
        rospy.Subscriber(
                'enemy_pos_from_score', Float32MultiArray, self.epfs_callback)
        rospy.Subscriber(
                'my_pose', MyPose, self.my_pose_callback)

    def reset(self):
        self.is_stop_receive = False

        self.client.cancel_goal()

        self.pub_vel()

    def is_data_exists(self):
        return ((self.score is not None) and
                (self.enemy_pos_from_score is not None) and
                (self.my_pose is not None))

    def stop_callback(self, data):
        if data.data:
            self.is_stop_receive = True

    def score_callback(self, data):
        self.score = data.data

    def epfs_callback(self, data):
        self.enemy_pos_from_score = data.data

    def my_pose_callback(self,data):
        self.my_pose = data

    def check_stop(self):
        return self.is_stop_receive

    def check_score(self):
        return [i for i, e in enumerate(self.score) if e > 0]

    def check_enemy_pos_from_score(self):
        return self.enemy_pos_from_score

    def check_my_pose(self):
        return self.my_pose

    def check_client(self):
        return self.client.get_state()

    def set_goal(self, x, y, yaw):
        my_move_base.setGoal(self.client, x / 1000.0, y / 1000.0, yaw)

    def pub_vel(self, x=0, y=0, a=0):
        msg = Twist()

        msg.linear.x = x
        msg.linear.y = y

        msg.angular.z = a

        self.cmd_vel.publish(msg)


class Selecting(smach.State):
    def __init__(self, func):
        outcomes = ['success', 'end']

        keys = ['target']

        smach.State.__init__(
                self, outcomes=outcomes, input_keys=keys, output_keys=keys)

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
        mypos = self.func.check_my_pose()
        enemy = self.func.check_enemy_pos_from_score()

        costs = [x.bcost for x in MK_INFOS]

        for i in self.func.check_score():

            if i < len(MK_INFOS):
                costs[i] += K_MY_MARKER

        for i in range(len(costs)):
            costs[i] += self.distance(MK_INFOS[i].point, mypos) * K_MY_POSE

            for j in MK_INFOS[i].nears:
                costs[i] += enemy[j] * K_ENEMY_POS_FROM_SCORE

        cost = min(costs)

        if cost < K_MY_MARKER:
            userdata.target = costs.index(cost)

    def distance(self, point, my_pose):
        dx = point[0] - my_pose.pos.x * 1000
        dy = point[1] - my_pose.pos.y * 1000

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
    def __init__(self, point, bcost, nears):
        self.point = point
        self.bcost = bcost
        self.nears = nears


RATE = 10


TIMEOUT_READING = 5


VEL_SPIN = (0, 0, math.pi / 2)


K_MY_MARKER = 10000

K_ENEMY_POS_FROM_SCORE = 1

K_MY_POSE = 0.01


MK_INFOS = eval("""
[
        MkInfo((-530,  800, -math.pi / 2), 0.00, [ 2,  3]),
        MkInfo(( 530,  800, -math.pi / 2), 0.00, [ 4,  5]),
        MkInfo((-530,  260,  math.pi / 2), 0.25, [ 7,  8]),
        MkInfo(( 530,  260,  math.pi / 2), 0.25, [ 9, 10]),
        MkInfo((   0,  340, -math.pi / 2), 0.50, [ 8,  9]),
        MkInfo((-340,    0,  0),           0.50, [ 8, 14]),
        MkInfo(( 340,    0,  math.pi),     0.50, [ 9, 15]),
        MkInfo((   0, -340,  math.pi / 2), 0.50, [14, 15]),
        MkInfo((-530, -260, -math.pi / 2), 0.25, [13, 14]),
        MkInfo(( 530, -260, -math.pi / 2), 0.25, [15, 16]),
        MkInfo((-530, -800,  math.pi / 2), 0.00, [18, 19]),
        MkInfo(( 530, -800,  math.pi / 2), 0.00, [20, 21])
]
""")
