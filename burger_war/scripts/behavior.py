#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import math
import roslib
import rospy
import smach
import smach_ros
import behavior_XXX
import behavior_escape, behavior_attack, behavior_disturb


<<<<<<< HEAD
from std_msgs.msg import Float32MultiArray, String
=======
from std_msgs.msg import Float32MultiArray
>>>>>>> origin/dev_saedo


class behavior:
    def __init__(self):
        pass

    def run(self):
        sm_top = smach.StateMachine(outcomes=['outcome'])
        # 各動作を追加
        with sm_top:
            #次の動作を決定する動作
            smach.StateMachine.add('Strategy', behavior_strategy(),
                               transitions={'escape' :'Escape',
                                            'attack' :'Attack',
                                            'disturb':'Disturb',
                                            'end':'outcome'
                                            })
            #逃げる動作
            smach.StateMachine.add('Escape', behavior_escape.bevavior_escape(),
#                                transitions={'outcome':'Escape'})
                               transitions={'outcome':'Strategy'})
            #点数を取りに行く動作
            smach.StateMachine.add('Attack', behavior_attack.behavior_attack(),
                               transitions={'outcome':'Strategy'})
            #妨害する動作
            smach.StateMachine.add('Disturb', behavior_disturb.bevavior_disturb(),
                               transitions={'outcome':'Strategy'})


        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
        sis.start()

        outcome = sm_top.execute()

class behavior_strategy(smach.State):
    def __init__(self):
        #このステートの返り値リストを定義。
        smach.State.__init__(self, outcomes=['escape','attack','disturb','end'])
        #次の状態を決めるためのダミー変数
        self.dummy_counter=0
        # bot name 
        robot_name=rospy.get_param('~robot_name')
        self.name = robot_name
        # sub
        self.strategy = None
        self.strategy_sub = rospy.Subscriber('/{}/strategy'.format(self.name), String, self.strategy_callback)

    def strategy_callback(self, data):
        self.strategy = data
        print('behavior strategy', self.strategy)



    def execute(self,userdata):
        if self.strategy != None:
            return self.strategy
        else:
            return 'attack'

        """
        #次の状態を決める(今は順番)
        self.dummy_counter+=1

        if(self.dummy_counter>=3):
            self.dummy_counter=0

        if(self.dummy_counter==0):
            return 'escape'
        elif(self.dummy_counter==1):
            # return 'escape'
            return 'attack'
        else:
            return 'disturb'
        """



        
def main(args):
    rospy.init_node('behavior', anonymous=True)
    ra = behavior()
    print('[behavior]initialized')
    ra.run()

if __name__=='__main__':
    main(sys.argv)
