#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ====================
# セルフプレイ部
# ====================
from game import State
from pv_mcts import pv_mcts_scores
from dual_network import DN_OUTPUT_SIZE
from datetime import datetime
from tensorflow.keras.models import load_model
from tensorflow.keras import backend as K
from pathlib import Path
from std_msgs.msg import Float32MultiArray, Time, String, Bool, Int32MultiArray, Int32
from geometry_msgs.msg import Point
from burger_war.msg import MyPose 
import numpy as np
import pickle
import os
import subprocess
import rospy
import time
import random

# パラメータの準備
SP_GAME_COUNT = 100 # セルフプレイを行うゲーム数（本家は25000）
SP_TEMPERATURE = 10.0 # ボルツマン分布の温度パラメータ

class SelfPlay():
    def __init__(self):
        self.name = ''
        # sub
        self.sub_enemy_pos_from_score = rospy.Subscriber('/{}/enemy_pos_from_score'.format(self.name), Float32MultiArray, self.enemy_pos_from_score_callback)
        self.enemy_pos_from_score=Float32MultiArray()

        self.sub_score = rospy.Subscriber('/{}/score'.format(self.name),Int32MultiArray,self.score_callback)
        self.score = []
        
        self.sub_rem_time=rospy.Subscriber('rem_time',Time,self.rem_time_callback)
        self.rem_time = Time()

        self.sub_enemy_pos_from_lider = rospy.Subscriber('/{}/enemy_pos_from_lider'.format(self.name), Point, self.enemy_pos_from_lider_callback)
        self.enemy_pos_from_lider = Point()
        
        self.sub_rem_time=rospy.Subscriber('rem_time',Time,self.rem_time_callback)
        self.rem_time = Time()

        self.sub_my_pose = rospy.Subscriber('/{}/my_pose'.format(self.name), MyPose, self.my_pose_callback)
        self.my_pose = MyPose()

        self.sub_enemy_pose_from_camera = rospy.Subscriber('/{}/enemy_pose_from_camera'.format(self.name), MyPose, self.enemy_pose_from_camera_callback)
        self.enemy_pose_from_camera = MyPose()

        # game State更新用
        self.my_pose_pos_list = [0.0]*3
        self.my_pose_ori_euler_list = [0.0]*3
        self.enemy_pos_from_lider_list = [0.0]*3
        self.enemy_pose_from_camera_pos_list = [0.0]*3
        self.enemy_pose_from_camera_ori_euler_list = [0.0]*3

        # シミュレーション起動確認用：
        self.is_topic_receive = False

        # pub: 次行動決定値
        self.pub_strategy = rospy.Publisher('/{}/ai_next_decition'.format(self.name), Int32, queue_size=1)


    def score_callback(self, data):
        self.score = data.data

    def enemy_pos_from_score_callback(self,data):
        self.enemy_pos_from_score=data
    
    def rem_time_callback(self, data):
        self.rem_time = data.data.secs
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



    # 学習データの保存
    def write_data(self, history):
        now = datetime.now()
        os.makedirs('./data/', exist_ok=True) # フォルダがない時は生成
        path = './data/{:04}{:02}{:02}{:02}{:02}{:02}.history'.format(
            now.year, now.month, now.day, now.hour, now.minute, now.second)
        with open(path, mode='wb') as f:
            pickle.dump(history, f)

    # 1ゲームの実行
    def play(self, model):
        global SP_TEMPERATURE
        # 学習データ
        history = []

        # 状態の生成
        state = State()
        print('init State, score', state.score_list)
        # 状態の更新頻度: Hz
        r=rospy.Rate(0.5)
        # rostopicの取得で状態を変換させ学習データを蓄積
        while not rospy.is_shutdown():
            # ゲーム終了時
            if state.is_done():
                break

            # 合法手の確率分布の取得
            scores = pv_mcts_scores(model, state, SP_TEMPERATURE)
            # 確率分布を散らす：状態で変化させなくて良いの？
            SP_TEMPERATURE = random.randrange(10)
            # 学習データに状態と方策を追加
            policies = [0] * DN_OUTPUT_SIZE
            for action, policy in zip(state.legal_actions(), scores):
                policies[action] = policy
            history.append([state.input_state_array(), policies, None])

            # 行動の取得
            action = np.random.choice(state.legal_actions(), p=scores)
            print('action', action, 'scores', scores)
            self.pub_strategy.publish(action)
            # 次の状態の取得
            state = state.next(self.score, self.enemy_pos_from_lider_list, self.rem_time, 
                                self.my_pose_pos_list, self.my_pose_ori_euler_list, self.enemy_pose_from_camera_pos_list, self.enemy_pose_from_camera_ori_euler_list)
            

            r.sleep()

        # 学習データに価値を追加
        return history


    # シミュレーションの起動と開始を確認
    def start_simulation(self):
        # ジャッジサーバ起動
        judge_path_sh = './scripts/sim_with_judge.sh'
        ros_source = '../../devel/setup.bash'
        subprocess.run(['gnome-terminal', '--', 'bash', judge_path_sh])
        time.sleep(10)
        # スクリプト起動
        script_path_sh = './scripts/start.sh'
        subprocess.run(['gnome-terminal', '--', 'bash', script_path_sh, '-l', '3'])
        # シミュレーション起動するまで待機
        count = 0
        while self.is_topic_receive is False:
            time.sleep(1)
            print('self.is_topic_receive', self.is_topic_receive)
            count=count+1
            if count > 20:
                return False
        return True
    

    # シミュレーションの起動と開始を確認
    def pkill_simulation(self):
        # スクリプト停止
        path_sh = './scripts/pkill.sh'
        subprocess.run(['sh', path_sh])
        # シミュレーション停止
        time.sleep(10)
        return
        

    # セルフプレイ
    def self_play(self):
        # 学習データ
        history = []

        # ベストプレイヤーのモデルの読み込み
        print('Read best model')
        model = load_model('./burger_war/scripts/model/best.h5')
        # model = load_model('./burger_war/scripts/model/latest.h5')

        # 複数回のゲームの実行
        print('Start Simulation: ', SP_GAME_COUNT)
        for i in range(SP_GAME_COUNT):
            # スクリプト起動してシミュレーション開始
            print('Execute Simulation')
            execute_result = self.start_simulation()
            if execute_result == False:
                time.sleep(3)
                self.pkill_simulation()
                continue
            # 1ゲームの実行: play内でrostopic取得
            print('Play Game')
            h = self.play(model)
            history.extend(h)

            print('Kill Simulation')
            # シミュレーション停止
            self.pkill_simulation()
            self.__init__()
            # 出力
            print('\rSelfPlay {}/{}'.format(i+1, SP_GAME_COUNT))
        print('')

        # 学習データの保存
        self.write_data(history)

        # モデルの破棄
        K.clear_session()
        del model

# 動作確認
if __name__ == '__main__':
    rospy.init_node("self_play")
    self_play = SelfPlay()
    self_play.self_play()
