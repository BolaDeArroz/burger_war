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
from std_msgs.msg import Bool, Float32MultiArray


class bevavior_attack(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['outcome'])

        # 内部のステートマシンsm_subを定義
        # この内部ステートマシンは,outcome
        self.sm_sub = smach.StateMachine(outcomes=['outcome'])

        # sm_subにステートを追加
        # ステートにできるのは、smach.Stateを継承したクラスのみ。(だと思う)
        # smach.StateMachine.addでステートを追加する
        # smach.StateMachine.add(ステート名,クラス名(=実体),
        #                        transitions={'ステートの返り値1':返り値1の時に遷移するステート名,
        #                                     'ステートの返り値2':返り値2の時に遷移するステート名}
        # ってな感じで、遷移先が複数あるならtransitionsをどんどん追加していく
        with self.sm_sub:
            # 最初にaddしたステートが開始時のステート
            smach.StateMachine.add('Selecting', Selecting(), transitions={
                'success': 'Moving',
                'end': 'outcome'  # ←sm_sub自体の終了
            })
            smach.StateMachine.add('Moving', Moving(), transitions={
                'success': 'Reading',
                'fail': 'Selecting',
                'read': 'Selecting',
                'end': 'outcome'  # ←sm_sub自体の終了
            })
            smach.StateMachine.add('Reading', Reading(), transitions={
                'success': 'Selecting',
                'timeout': 'Selecting',
                'end': 'outcome'  # ←sm_sub自体の終了
            })

        # 下2行はsmach_viewerでステートを確認するために必要
        sis = smach_ros.IntrospectionServer(
                'server_name', self.sm_sub, '/SM_ATTACK')
        sis.start()

    def execute(self, userdata):
        # 内部のステートマシンの実行
        self.sm_sub.execute()

        return 'outcome'


class SubBehaviorBase(smach.State):

    def __init__(self, outcomes, input_keys=[], output_keys=[]):
        # このステートの返り値リストを定義。
        smach.State.__init__(
                self,
                outcomes=outcomes,
                input_keys=input_keys,
                output_keys=output_keys)

        # 停止トピックを受け取るための定義。
        rospy.Subscriber('state_stop', Bool, self.stop_callback)
        self.is_stop_receive = False

    def stop_callback(self, data):
        self.is_stop_receive = True

    def check_stop(self):
        # 処理中の終了できる箇所で、停止トピックを受け取ったか確認。
        if self.is_stop_receive:
            self.is_stop_receive = False
            return True

        return False


class Selecting(SubBehaviorBase):

    def __init__(self):
        super(Selecting, self).__init__(['success', 'end'], [], ['target'])

        self.score = []

        rospy.Subscriber('pub_score', Float32MultiArray, self.score_callback)

    def execute(self, userdata):
        r = rospy.Rate(60)

        while not rospy.is_shutdown():
            if self.check_stop():
                return 'end'

            target = self.select()

            if target is not None:
                userdata.target = target

                return 'success'

            r.sleep()

        return 'end'

    def score_callback(self, data):
        self.score = data.data

    def select(self):
        return self.score.index(1) if 1 in self.score else None


class Moving(SubBehaviorBase):

    def __init__(self):
        super(Moving, self).__init__(['success', 'fail', 'read', 'end'], ['target'], [])

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, userdata):
        r = rospy.Rate(60)

        x, y, yaw = POINTS[userdata.target]

        self.set_goal(x, y, yaw)

        while not rospy.is_shutdown():
            if self.check_stop():
                return 'end'

            next = self.check_client_state()

            if next != '':
                return next

            r.sleep()

        return 'end'

    def set_goal(self, x, y, yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)

        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)

    def check_client_state(self):
        state = self.client.get_state()

        if state == actionlib_msgs.msg.GoalStatus.PENDING:
            return ''

        if state == actionlib_msgs.msg.GoalStatus.ACTIVE:
            return ''

        if state == actionlib_msgs.msg.GoalStatus.RECALLED:
            return ''

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

        return ''


class Reading(SubBehaviorBase):

    def __init__(self):
        super(Reading, self).__init__(['success', 'timeout', 'end'], [], [])

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
