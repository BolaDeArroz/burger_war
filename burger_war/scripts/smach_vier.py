#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import math
import roslib
import rospy
import smach

class bevavior_escape(smach.State):
    def __init__(self):
        # bot name 
        robot_name=''
        self.name = robot_name
        smach.State.__init__(self, outcomes=['outcome'])

        
    def execute(self,userdata):    
        #内部のステートマシン
        sm_sub = smach.StateMachine(outcomes=['outcome'])
        # Open the container
        with sm_sub:
        # Add states to the container
            smach.StateMachine.add('CalcEnemyPos', CalcEnemyPos(), 
                                  transitions={'outcome1':'DecideEscapePoint', 
                                               'outcome2':'Wait'})
            smach.StateMachine.add('DecideEscapePoint', DecideEscapePoint(), 
                                  transitions={'outcome1':'MoveToEscapePoint'})
            smach.StateMachine.add('MoveToEscapePoint', MoveToEscapePoint(), 
                                  transitions={'outcome1':'CalcEnemyPos'})
            smach.StateMachine.add('Wait', Wait(), 
                                  transitions={'outcome1':'CalcEnemyPos'})    
        sm_sub.execute()
        return 'outcome'

class CalcEnemyPos(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self,userdata):
        #敵の位置を計算する処理
        rospy.sleep(3)
        return 'outcome1'

class DecideEscapePoint(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self,userdata):
        #逃げる位置を決める処理
        rospy.sleep(3)
        return 'outcome1'

class MoveToEscapePoint(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self,userdata):
        #逃げる処理
        rospy.sleep(3)
        return 'outcome1'

class Wait(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self,userdata):
        #待つ処理
        rospy.sleep(3)
        return 'outcome1'