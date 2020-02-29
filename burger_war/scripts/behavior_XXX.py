#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import math
import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool 

class bevavior_XXX(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['outcome'])
        #内部のステートマシンsm_subを定義
        #この内部ステートマシンは,outcome
        self.sm_sub = smach.StateMachine(outcomes=['outcome'])
        
        # sm_subにステートを追加
        # ステートにできるのは、smach.Stateを継承したクラスのみ。(だと思う)
        # smach.StateMachine.addでステートを追加する
        # smach.StateMachine.add(ステート名,クラス名(=実体),
        #                        transitions={'ステートの返り値1':返り値1の時に遷移するステート名,
        #                                     'ステートの返り値2':返り値2の時に遷移するステート名}
        #ってな感じで、遷移先が複数あるならtransitionsをどんどん追加していく
        with self.sm_sub:
            #最初にaddしたステートが開始時のステート
            smach.StateMachine.add('State_A', State_A(), 
                                  transitions={'outcome1':'State_B', 
                                               'outcome2':'outcome'}) #←sm_sub自体の終了
            smach.StateMachine.add('State_B', State_B(), 
                                  transitions={'outcome1':'State_A',
                                               'outcome2':'outcome'}) #←sm_sub自体の終了

        #下2行はsmach_viewerでステートを確認するために必要
        sis = smach_ros.IntrospectionServer('server_name', self.sm_sub, '/SM_XXX')
        sis.start()
        
    def execute(self,userdata):    
        #内部のステートマシンの実行
        self.sm_sub.execute()
        return 'outcome'



class State_A(smach.State):

    def __init__(self):
        #このステートの返り値リストを定義。
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        
        #停止トピックを受け取るための定義。 
        robot_name=''
        self.name = robot_name
        self.stop_sub = rospy.Subscriber('/{}/state_stop'.format(self.name), Bool, self.stop_callback)
        self.is_stop_receive=False

    def execute(self,userdata):
        #ステート1の処理
        rospy.sleep(3)

        # 処理中の終了できる箇所で、停止トピックを受け取ったか確認。
        if self.is_stop_receive:
            self.is_stop_receive=False
            # 受け取っていたら終了。(終了処理とかもここに)
            return 'outcome2'

        return 'outcome1'


    def stop_callback(self,data):
        self.is_stop_receive=True


class State_B(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        
        #停止トピックを受け取るための定義。 
        robot_name=''
        self.name = robot_name
        self.stop_sub = rospy.Subscriber('/{}/state_stop'.format(self.name), Bool, self.stop_callback)
        self.is_stop_receive=False

    def execute(self,userdata):
        #ステート1の処理
        rospy.sleep(3)

        # 処理中の終了できる箇所で、停止トピックを受け取ったか確認。
        if self.is_stop_receive:
            self.is_stop_receive=False
            # 受け取っていたら終了。(終了処理とかもここに)
            return 'outcome2'

        return 'outcome1'


    def stop_callback(self,data):
        self.is_stop_receive=True
