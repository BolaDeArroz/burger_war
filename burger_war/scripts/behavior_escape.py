#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import math
import roslib
import rospy
import rosparam
import smach
import smach_ros
import tf
import actionlib
import actionlib_msgs

from std_msgs.msg import Bool,Float32MultiArray
from geometry_msgs.msg import Point,Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from burger_war.msg import MyPose 

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
                                               'is_NearEnemyFound':'GoToEscapePoint',
                                               'is_receiveStopSig':'outcome'},
                                  remapping={'enemy_pos_in':'enemy_pos',
                                             'enemy_pos_out':'enemy_pos'})

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
        self.sub_enemy_pos_from_lider = rospy.Subscriber('/{}/enemy_pos_from_lider_last'.format(self.name), Point, self.enemy_pos_from_lider_callback)
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

    def enemy_pos_from_lider_callback(self,data):
        self.enemy_pos_from_lider["enemy_pos"]=data
        self.enemy_pos_from_lider["is_topic_receive"]=True

    def my_pose_callback(self,data):
        self.my_pose=data

    def get_distance(self,x1, y1, x2, y2):
        d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return d

    def __init__(self):
        smach.State.__init__(self,  outcomes=['is_Gone','is_NearEnemyFound','is_receiveStopSig'],
                                    input_keys=['enemy_pos_in'],
                                    output_keys=['enemy_pos_out'])

        robot_name=''
        self.name = robot_name

        #停止トピックを受け取るための定義。         
        self.sub_stop = rospy.Subscriber('/{}/state_stop'.format(self.name), Bool, self.stop_callback)
        self.is_stop_receive=False

        # liderから敵位置推定トピックを受け取るための定義
        self.sub_enemy_pos_from_lider = rospy.Subscriber('/{}/enemy_pos_from_lider'.format(self.name), Point, self.enemy_pos_from_lider_callback)
        self.enemy_pos_from_lider={"enemy_pos":Point(),"is_topic_receive":False}

        # 自己位置トピックを受け取るための定義
        self.sub_my_pose = rospy.Subscriber('/{}/my_pose'.format(self.name), MyPose, self.my_pose_callback)
        self.my_pose=MyPose()

        #Move base クライアント
        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        # 移動途中に敵を発見した時に、目的地を切り替えるしきい値[m](前回の敵座標-発見した敵座標)
        self.CHANGE_ESCAPE_POS_TH=0.50


    def calc_escape_pos_v1(self,x,y):
            #ゴール時の方向はマップ中心を向く
            return -x,-y,math.atan2(y,x)-math.pi

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
        #ゴール時の方向はマップ中心を向く
        return escape_pos_list[idx]["x"],escape_pos_list[idx]["y"],math.atan2(escape_pos_list[idx]["y"],escape_pos_list[idx]["x"])-math.pi

    def calc_escape_pos_v3(self,x,y):
        #マップを8分割45°区切りで分けて、敵の座標によってその反対側の決められた地点に逃げる(点数取れる位置版)
        escape_pos_list=[{"x": 0.0,"y": 0.5, "yaw":-math.pi/2},\
                        {"x":-0.53,"y": 0.8, "yaw":-math.pi/2}, \
                        {"x":-0.50,"y": 0.0, "yaw":0}, \
                        {"x":-0.53,"y":-0.8, "yaw":math.pi/2},\
                        {"x": 0.00,"y":-0.5, "yaw":math.pi/2},\
                        {"x": 0.53,"y":-0.8, "yaw":math.pi/2},\
                        {"x": 0.50,"y": 0.0, "yaw":math.pi},\
                        {"x": 0.53,"y": 0.8,"yaw":-math.pi/2}\
        ]
        idx=(int(round(math.degrees(math.atan2(-x,y))/45))+4) % len(escape_pos_list) 
        #ゴール時の方向は決められた方向を向く
        return escape_pos_list[idx]["x"],escape_pos_list[idx]["y"],escape_pos_list[idx]["yaw"]

    def calc_escape_pos_v4(self,en_x,en_y,my_x,my_y):
        #マップを8分割45°区切りで分けて、敵の座標と自分の座標によって決められた位置に逃げる
        #敵より自分のほうが近い地点の中で敵から最も遠い地点
        escape_pos_list=[{"x": 0.0,"y": 1.3},\
                        {"x":-0.5,"y": 0.8},\
                        {"x":-1.3,"y": 0.0},\
                        {"x":-0.5,"y":-0.8},\
                        {"x": 0.0,"y":-1.3},\
                        {"x": 0.5,"y":-0.8},\
                        {"x": 1.3,"y": 0.0},\
                        {"x": 0.5,"y": 0.8}
        ]
        #敵から地点の距離
        escape_dist_from_enemy=[self.get_distance(en_x,en_y,pos["x"],pos["y"]) for pos in escape_pos_list ]
        #print("enemy_=",escape_dist_from_enemy)
        #自分から地点の距離
        escape_dist_from_my=   [self.get_distance(my_x,my_y,pos["x"],pos["y"]) for pos in escape_pos_list ]
        #print("my_=",escape_dist_from_my)
        #敵-自分
        compare_dist=          [en-my for en,my in zip(escape_dist_from_enemy,escape_dist_from_my)] 
        print("en_my_=",compare_dist)
        
        #候補を自分のほうが近い地点
        #      現在地点から一定以上離れた地点(スタック防止のため)
        # のみに絞る
        enable_pos_list=[pos for comp_dist,dist_from_my,pos in zip(compare_dist,escape_dist_from_my,escape_pos_list) if comp_dist>=0 and dist_from_my >=0.50]
        if len(enable_pos_list) >=1: 
            enable_pos_dist=[self.get_distance(en_x,en_y,pos["x"],pos["y"]) for pos in enable_pos_list]
            idx=enable_pos_dist.index(max(enable_pos_dist))
        else:#候補座標無い場合 
            #v2と同じ座標
            idx=(int(round(math.degrees(math.atan2(-en_x,en_y))/45))+4) % len(escape_pos_list) 
        #ゴール時の方向はマップ中心を向く
        return enable_pos_list[idx]["x"],enable_pos_list[idx]["y"],math.atan2(enable_pos_list[idx]["y"],enable_pos_list[idx]["x"])-math.pi


    def execute(self,userdata):
        #パラメータ初期化
        self.enemy_pos_from_lider={"enemy_pos":Point(),"is_topic_receive":False}

        #移動時の向きを逃走用(後ろ向き)に設定
        rosparam.set_param("/move_base/GlobalPlanner/orientation_mode", "4")

        #逃げる位置計算
        enemy_pos=userdata.enemy_pos_in
        print("enemy_pos",enemy_pos)
        
        #escape_pos_x,escape_pos_y,escape_yaw=self.calc_escape_pos_v1(enemy_pos.x,enemy_pos.y)#中心挟んで相手の反対地点に逃げるパターン
        #escape_pos_x,escape_pos_y,escape_yaw=self.calc_escape_pos_v2(enemy_pos.x,enemy_pos.y)#相手の位置によって反対側の決まった地点に逃げるパターン
        #escape_pos_x,escape_pos_y,escape_yaw=self.calc_escape_pos_v3(enemy_pos.x,enemy_pos.y)#相手の位置によって反対側の決まった地点に逃げるパターン
        escape_pos_x,escape_pos_y,escape_yaw=self.calc_escape_pos_v4(enemy_pos.x,enemy_pos.y,self.my_pose.pos.x,self.my_pose.pos.y)#相手の位置によって反対側の決まった地点に逃げるパターン
       
        #print(escape_pos_x,escape_pos_y,escape_yaw)

        #ゴール設定
        my_move_base.setGoal(self.move_base_client,escape_pos_x,escape_pos_y,escape_yaw)
        
        # rospy終了か、ゴールに着いたらループ抜ける。
        self.is_stop_receive=False
        r = rospy.Rate(5)
        while (not rospy.is_shutdown()) and \
                self.move_base_client.get_state() in [  actionlib_msgs.msg.GoalStatus.ACTIVE,\
                                                        actionlib_msgs.msg.GoalStatus.PENDING]:

            #逃げる途中で,ストップトピックを受け取った場合
            if self.is_stop_receive == True:
                #ループ抜ける
                #目的地設定キャンセル
                self.move_base_client.cancel_goal()
                #移動停止
                for _ in range(5):
                    self.vel_pub.publish(Twist())
                    r.sleep()
                self.is_stop_receive=False
                rosparam.set_param("/move_base/GlobalPlanner/orientation_mode", "1")#(None=0, Forward=1, Interpolate=2, ForwardThenInterpolate=3, Backward=4, Leftward=5, Rightward=6)
                return 'is_receiveStopSig'

            #逃げる途中で、敵と遭遇した場合
            if self.enemy_pos_from_lider["is_topic_receive"]==True:
                #前回の敵位置との距離を計算
                dist=self.get_distance(                          enemy_pos.x,                             enemy_pos.y,\
                                    self.enemy_pos_from_lider["enemy_pos"].x,self.enemy_pos_from_lider["enemy_pos"].y)
                # print("dist",dist)
                if dist >= self.CHANGE_ESCAPE_POS_TH: #しきい値を上回っていた場合
                    userdata.enemy_pos_out=self.enemy_pos_from_lider["enemy_pos"]
                    self.move_base_client.cancel_goal()
                    rosparam.set_param("/move_base/GlobalPlanner/orientation_mode", "1")#(None=0, Forward=1, Interpolate=2, ForwardThenInterpolate=3, Backward=4, Leftward=5, Rightward=6)
                    return 'is_NearEnemyFound' #抜ける。再計算されて、目的地が変更される。

            
            #TODO:ゴールが壁の中になった時の対応
            r.sleep()
        #目的地についた場合
        rosparam.set_param("/move_base/GlobalPlanner/orientation_mode", "1")#(None=0, Forward=1, Interpolate=2, ForwardThenInterpolate=3, Backward=4, Leftward=5, Rightward=6)
        return 'is_Gone'


