#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import math
import roslib
import rospy
import smach
import smach_ros
import tf
import actionlib
import actionlib_msgs

from std_msgs.msg import Bool,Float32MultiArray
from geometry_msgs.msg import Point,Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import my_move_base

class bevavior_escape(smach.State):
    def __init__(self):
        #robot name 
        robot_name=''
        self.name = robot_name
       
        smach.State.__init__(self, outcomes=['outcome'])
        #内部のステートマシンsm_subを定義
        #この内部ステートマシンは,outcome
        self.sm_sub = smach.StateMachine(outcomes=['outcome'])
        self.sm_sub.userdata.enemy_pos=Point()

        # Open the container
        with self.sm_sub:
        # Add states to the container
            smach.StateMachine.add('CalcEnemyPos', CalcEnemyPos(), 
                                  transitions={'is_EnemyFound'    :'GoToEscapePoint', 
                                               'is_receiveStopSig':'outcome'},
                                  remapping={'enemy_pos_out':'enemy_pos'})
            smach.StateMachine.add('GoToEscapePoint', GoToEscapePoint(), 
                                  transitions={'is_Gone'          :'CalcEnemyPos',
                                               'is_receiveStopSig':'outcome'},
                                  remapping={'enemy_pos_in':'enemy_pos'})

        #下2行はsmach_viewerでステートを確認するために必要                                      
        sis = smach_ros.IntrospectionServer('server_name', self.sm_sub, 'SM_ESCAPE')
        sis.start()        
        
    def execute(self,userdata):    
        #内部のステートマシンの実行
        self.sm_sub.execute()
        return 'outcome'




class CalcEnemyPos(smach.State):

    def stop_callback(self,data):
        self.is_stop_receive=True

    def enemy_pos_from_lider_callback(self,data):
        self.enemy_pos_from_lider["enemy_pos"]=data
        self.enemy_pos_from_lider["is_topic_receive"]=True
    
    def enemy_pos_from_score_callback(self,data):
        self.enemy_pos_from_score=data
        
        
    def __init__(self):
        #このステートの返り値リストを定義。
        smach.State.__init__(self,  outcomes=['is_EnemyFound','is_receiveStopSig'],
                                    output_keys=['enemy_pos_out'])

        robot_name=''
        self.name = robot_name
        #停止トピックを受け取るための定義。         
        self.sub_stop = rospy.Subscriber('/{}/state_stop'.format(self.name), Bool, self.stop_callback)
        self.is_stop_receive=False

        # liderから敵位置推定トピックを受け取るための定義
        self.sub_enemy_pos_from_lider = rospy.Subscriber('/{}/enemy_pos_from_lider'.format(self.name), Point, self.enemy_pos_from_lider_callback)
        self.enemy_pos_from_lider={"enemy_pos":Point(),"is_topic_receive":False}

        # scoreから敵位置推定トピックを受け取るための定義
        self.sub_enemy_pos_from_score = rospy.Subscriber('/{}/enemy_pos_from_score'.format(self.name), Float32MultiArray, self.enemy_pos_from_score_callback)
        self.enemy_pos_from_score=Float32MultiArray()

    def execute(self,userdata):


        #パラメータ初期化
        self.enemy_pos_from_lider={"enemy_pos":Point(),"is_topic_receive":False}


        # rospy終了か、停止トピックを受け取ったらループ抜ける。
        r=rospy.Rate(5)
        self.is_stop_receive=False
        while (not rospy.is_shutdown()) and (self.is_stop_receive==False):
            #敵の位置を推測したらループを抜ける
            #TODO:score情報からの敵位置
            if self.enemy_pos_from_lider["is_topic_receive"]:
                userdata.enemy_pos_out=self.enemy_pos_from_lider["enemy_pos"]
                return 'is_EnemyFound'
            
            r.sleep()
        
        self.is_stop_receive=False
        return 'is_receiveStopSig'


class GoToEscapePoint(smach.State):

    def stop_callback(self,data):
        self.is_stop_receive=True


    def __init__(self):
        smach.State.__init__(self,  outcomes=['is_Gone','is_receiveStopSig'],
                                    input_keys=['enemy_pos_in'])

        robot_name=''
        self.name = robot_name

        #停止トピックを受け取るための定義。         
        self.sub_stop = rospy.Subscriber('/{}/state_stop'.format(self.name), Bool, self.stop_callback)
        self.is_stop_receive=False

        #Move base クライアント
        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

    def calc_escape_pos_v1(self,x,y):
            return -x,-y

    def calc_escape_pos_v2(self,x,y):
        #マップを8分割45°区切りで分けて、敵の座標によってその反対側の決められた地点に逃げる
        escape_pos_list=[{"x": 0.0,"y": 1.3},\
                        {"x":-0.5,"y": 0.8},\
                        {"x":-1.3,"y": 0.0},\
                        {"x":-0.5,"y":-0.8},\
                        {"x": 0.0,"y":-1.3},\
                        {"x": 0.5,"y":-0.8},\
                        {"x": 1.3,"y": 0.0},\
                        {"x": 0.5,"y": 0.8}
        ]
        idx=(int(round(math.degrees(math.atan2(-x,y))/45))+4) % len(escape_pos_list) 
        return escape_pos_list[idx]["x"],escape_pos_list[idx]["y"],


    def execute(self,userdata):


        #逃げる位置計算
        enemy_pos=userdata.enemy_pos_in
        print("enemy_pos",enemy_pos)
        #中心挟んで相手の反対地点に逃げるパティーン
        #escape_pos_x,escape_pos_y=self.calc_escape_pos_v1(enemy_pos.x,enemy_pos.y)
        
        #相手の位置によって反対側の決まった地点に逃げるパティーン
        escape_pos_x,escape_pos_y=self.calc_escape_pos_v2(enemy_pos.x,enemy_pos.y)
        
        #ゴール時の方向はマップ中心を向く用に変更
        escape_yaw=math.atan2(escape_pos_y,escape_pos_x)-math.pi
        #print(escape_pos_x,escape_pos_y)

        #ゴール設定
        #my_move_base.setGoal(self.move_base_client,escape_pos_x,escape_pos_y,escape_yaw)
        #DEBUG:
        my_move_base.setGoal(self.move_base_client,-0.45,-0.45,escape_yaw)
        
        # rospy終了か、ゴールに着いたらループ抜ける。
        self.is_stop_receive=False
        r = rospy.Rate(5)
        while (not rospy.is_shutdown()) and \
                self.move_base_client.get_state() in [  actionlib_msgs.msg.GoalStatus.ACTIVE,\
                                                        actionlib_msgs.msg.GoalStatus.PENDING]:

            #逃げる途中でストップトピックを受け取った場合も抜ける
            if self.is_stop_receive == True:
                #目的地設定キャンセル
                self.move_base_client.cancel_goal()
                #移動停止
                for _ in range(5):
                    self.vel_pub.publish(Twist())
                    r.sleep()
                self.is_stop_receive=False
                return 'is_receiveStopSig'
            
            #TODO:ゴールが壁の中になった時の対応

            r.sleep()
 
        
        #目的地についた場合
        return 'is_Gone'


