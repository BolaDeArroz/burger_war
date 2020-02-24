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
from std_msgs.msg import Bool, Int32MultiArray


class bevavior_attack(smach.State):
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
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.Subscriber('state_stop', Bool, self.stop_callback)
        rospy.Subscriber('pub_score', Int32MultiArray, self.score_callback)

    def stop_callback(self, data):
        self.is_stop_receive = True

    def score_callback(self, data):
        self.score = data.data

    def check_stop(self):
        result = self.is_stop_receive

        self.is_stop_receive = False

        return result

    def check_client(self):
        return self.client.get_state()

    def set_goal(self, x, y, yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        q = tf.transformations.quaternion_from_euler(0, 0, yaw)

        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)


class Selecting(smach.State):
    def __init__(self, func):
        smach.State.__init__(
                self,
                outcomes=['success', 'end'],
                out_keys=['target'])

        self.func = func

    def execute(self, userdata):
        r = rospy.Rate(60)

        while not rospy.is_shutdown():
            if self.func.check_stop():
                return 'end'

            target = self.select()

            if target is not None:
                userdata.target = target

                return 'success'

            r.sleep()

        return 'end'

    def select(self):
        return self.func.score.index(1) if 1 in self.func.score else None


class Moving(smach.State):
    def __init__(self, func):
        smach.State.__init__(
                self,
                outcomes=['success', 'fail', 'read', 'end'],
                in_keys=['target'])

        self.func = func

    def execute(self, userdata):
        r = rospy.Rate(60)

        x, y, yaw = POINTS[userdata.target]

        self.func.set_goal(x, y, yaw)

        while not rospy.is_shutdown():
            if self.func.check_stop():
                return 'end'

            result = self.check()

            if result is not None:
                return result

            r.sleep()

        return 'end'

    def check(self):
        state = self.func.check_client()

        if state == actionlib_msgs.msg.GoalStatus.PENDING:
            return None

        if state == actionlib_msgs.msg.GoalStatus.ACTIVE:
            return None

        if state == actionlib_msgs.msg.GoalStatus.RECALLED:
            return None

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

        return None


class Reading(smach.State):
    def __init__(self, func):
        smach.State.__init__(
                self,
                outcomes=['success', 'timeout', 'end'])

        self.func = func

    def execute(self, userdata):
        r = rospy.Rate(60)

        while not rospy.is_shutdown():
            return 'timeout'

            r.sleep()

        return 'end'


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
