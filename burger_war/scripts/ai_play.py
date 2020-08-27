#!/usr/bin/env python3
# -*- coding: utf-8 -*- 


from game import State
from pv_mcts import pv_mcts_action
from tensorflow.keras.models import load_model
from pathlib import Path
from std_msgs.msg import Float32MultiArray, Time, String, Bool, Int32MultiArray, Int32
from geometry_msgs.msg import Point
from burger_war.msg import MyPose 
import rospy

class AiPlay():
    def __init__(self):
        # ゲーム状態の生成
        self.ai_state = State()
        # ベストプレイヤーのモデルの読み込み
        model = load_model('/home/ctu/rhc_ws/src/burger_war/burger_war/scripts/model/latest.h5')
        # PV MCTSで行動選択を行う関数の生成
        self.next_action = pv_mcts_action(model, 0.0)

        # game State更新用
        self.score = [0]*18
        self.int_rem_time = 180
        self.my_pose_pos_list = [0.0]*3
        self.my_pose_ori_euler_list = [0.0]*3
        self.enemy_pos_from_lider_list = [0.0]*3
        self.enemy_pose_from_camera_pos_list = [0.0]*3
        self.enemy_pose_from_camera_ori_euler_list = [0.0]*3

        # シミュレーション起動確認用：
        self.is_topic_receive = False

        self.name = ''
        # sub
        self.sub_enemy_pos_from_score = rospy.Subscriber('/{}/enemy_pos_from_score'.format(self.name), Float32MultiArray, self.enemy_pos_from_score_callback)
        self.enemy_pos_from_score=Float32MultiArray()

        self.sub_score = rospy.Subscriber('/{}/score'.format(self.name),Int32MultiArray,self.score_callback)
        
        self.sub_rem_time=rospy.Subscriber('rem_time',Time,self.rem_time_callback)
        self.rem_time = Time()

        self.sub_enemy_pos_from_lider = rospy.Subscriber('/{}/enemy_pos_from_lider'.format(self.name), Point, self.enemy_pos_from_lider_callback)
        self.enemy_pos_from_lider = Point()
        

        self.sub_my_pose = rospy.Subscriber('/{}/my_pose'.format(self.name), MyPose, self.my_pose_callback)
        self.my_pose = MyPose()

        self.sub_enemy_pose_from_camera = rospy.Subscriber('/{}/enemy_pose_from_camera'.format(self.name), MyPose, self.enemy_pose_from_camera_callback)
        self.enemy_pose_from_camera = MyPose()

        # pub: 次行動決定値
        self.pub_strategy = rospy.Publisher('/{}/ai_next_decition'.format(self.name), Int32, queue_size=1)
    

    def score_callback(self, data):
        self.score = data.data

    def enemy_pos_from_score_callback(self,data):
        self.enemy_pos_from_score=data
    
    def rem_time_callback(self, data):
        self.rem_time = data
        self.int_rem_time = data.data.secs
        self.is_topic_receive = True

    def my_pose_callback(self,data):
        self.my_pose = data
        self.my_pose_pos_list = [self.my_pose.pos.x, self.my_pose.pos.y, self.my_pose.pos.z]
        self.my_pose_ori_euler_list = [self.my_pose.ori_euler.x, self.my_pose.ori_euler.y, self.my_pose.ori_euler.z]
        

    def enemy_pos_from_lider_callback(self,data):
        self.enemy_pos_from_lider = data
        self.enemy_pos_from_lider_list = [self.enemy_pos_from_lider.x, self.enemy_pos_from_lider.y, self.enemy_pos_from_lider.z]

    def enemy_pose_from_camera_callback(self, data):
        self.enemy_pose_from_camera = data
        self.enemy_pose_from_camera_pos_list = [self.enemy_pose_from_camera.pos.x, self.enemy_pose_from_camera.pos.y, self.enemy_pose_from_camera.pos.z]
        self.enemy_pose_from_camera_ori_euler_list = [self.enemy_pose_from_camera.ori_euler.x, self.enemy_pose_from_camera.ori_euler.y, self.enemy_pose_from_camera.ori_euler.z]
        



    def ai_strategy_run(self):
        r=rospy.Rate(0.5)
        
        while not rospy.is_shutdown():
            # ゲーム終了時
            if self.ai_state.is_done():
                return
            # 行動の取得
            action = self.next_action(self.ai_state)
            print('action', action)
            # publish next state
            self.pub_strategy.publish(action)
            # update ai state
            self.ai_state = self.ai_state.next(self.score, self.enemy_pos_from_lider_list, self.int_rem_time, 
                                self.my_pose_pos_list, self.my_pose_ori_euler_list, self.enemy_pose_from_camera_pos_list, self.enemy_pose_from_camera_ori_euler_list)
            r.sleep()
        



# 動作確認
if __name__ == '__main__':
    rospy.init_node("ai_play")
    ai_play = AiPlay()
    ai_play.ai_strategy_run()
    
    