#!/usr/bin/env python
# -*- coding: utf-8 -*-
# import sys
# import math
# import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool


class bevavior_attack(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['outcome'])

    def execute(self, userdata):
        # 内部のステートマシンsm_subを定義
        # この内部ステートマシンは,outcome
        sm_sub = smach.StateMachine(outcomes=['outcome'])

        # sm_subにステートを追加
        # ステートにできるのは、smach.Stateを継承したクラスのみ。(だと思う)
        # smach.StateMachine.addでステートを追加する
        # smach.StateMachine.add(ステート名,クラス名(=実体),
        #                        transitions={'ステートの返り値1':返り値1の時に遷移するステート名,
        #                                     'ステートの返り値2':返り値2の時に遷移するステート名}
        # ってな感じで、遷移先が複数あるならtransitionsをどんどん追加していく
        with sm_sub:
            # 最初にaddしたステートが開始時のステート
            smach.StateMachine.add('Selecting', Selecting(), transitions={
                'success': 'PathFinding',
                'end': 'outcome'  # ←sm_sub自体の終了
            })
            smach.StateMachine.add('PathFinding', PathFinding(), transitions={
                'success': 'Moving',
                'fail': 'Selecting',
                'end': 'outcome'  # ←sm_sub自体の終了
            })
            smach.StateMachine.add('Moving', Moving(), transitions={
                'success': 'Reading',
                'fail': 'PathFinding',
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
                'server_name', sm_sub, '/SM_ATTACK')
        sis.start()

        # 内部のステートマシンの実行
        sm_sub.execute()

        return 'outcome'


class SubBehaviorBase(smach.State):

    def __init__(self, outcomes):
        # このステートの返り値リストを定義。
        smach.State.__init__(self, outcomes=outcomes)

        # 停止トピックを受け取るための定義。
        robot_name = rospy.get_param('~robot_name')
        self.name = robot_name
        self.stop_sub = rospy.Subscriber(
                '/{}/state_stop'.format(self.name), Bool, self.stop_callback)
        self.is_stop_receive = False

    def stop_callback(self, data):
        self.is_stop_receive = True


class Selecting(SubBehaviorBase):

    def __init__(self):
        super().__init__(['success', 'end'])

    def execute(self, userdata):
        # ステート1の処理
        rospy.sleep(3)

        # 処理中の終了できる箇所で、停止トピックを受け取ったか確認。
        if self.is_stop_receive:
            self.is_stop_receive = False
            # 受け取っていたら終了。(終了処理とかもここに)
            return 'end'

        return 'end'


class PathFinding(SubBehaviorBase):

    def __init__(self):
        super().__init__(['success', 'fail', 'end'])

    def execute(self, userdata):
        # ステート1の処理
        rospy.sleep(3)

        # 処理中の終了できる箇所で、停止トピックを受け取ったか確認。
        if self.is_stop_receive:
            self.is_stop_receive = False
            # 受け取っていたら終了。(終了処理とかもここに)
            return 'end'

        return 'end'


class Moving(SubBehaviorBase):

    def __init__(self):
        super().__init__(['success', 'fail', 'read', 'end'])

    def execute(self, userdata):
        # ステート1の処理
        rospy.sleep(3)

        # 処理中の終了できる箇所で、停止トピックを受け取ったか確認。
        if self.is_stop_receive:
            self.is_stop_receive = False
            # 受け取っていたら終了。(終了処理とかもここに)
            return 'end'

        return 'end'


class Reading(SubBehaviorBase):

    def __init__(self):
        super().__init__(['success', 'timeout', 'end'])

    def execute(self, userdata):
        # ステート1の処理
        rospy.sleep(3)

        # 処理中の終了できる箇所で、停止トピックを受け取ったか確認。
        if self.is_stop_receive:
            self.is_stop_receive = False
            # 受け取っていたら終了。(終了処理とかもここに)
            return 'end'

        return 'end'
