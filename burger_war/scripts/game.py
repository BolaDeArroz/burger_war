#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
## ゲームの状態
- input
    enemy_pos_from_score
    score
    enemy_pos_from_lider
    rem_time
    my_pose
    enemy_pose_from_camera

- output
    次の状態：'attack', 'escape', 'disturb' 
"""

from dual_network import DN_INPUT_SHAPE


class State:
    # 初期化
    def __init__(self, score_list=None, enemy_pos_from_lider_list=None, rem_time=180, 
                 my_pose_pos_list=None, my_pose_ori_euler_list=None, enemy_pose_from_camera_pos_list=None, enemy_pose_from_camera_ori_euler_list=None):
        """
        取り敢えずconv2Dに入る形にする。3列×6行の入力
        1：自己点, 敵点, 残時間
        2：lider敵位置
        3：自己位置
        4：自己オイラー
        5：camera敵位置
        6：camera敵オイラー
        """
        """
        # 敵いる確率：0〜1の間の値
        self.enemy_pos_from_score_list =  enemy_pos_from_score_list if enemy_pos_from_score_list != None else [0.0]*24
        """
        
        # -1敵・0なし・1自分
        self.score_list = score_list if score_list != None else [0]*18
        # lider敵位置: x, y, z
        self.enemy_pos_from_lider_list =  enemy_pos_from_lider_list if enemy_pos_from_lider_list != None else [0.0]*3
        # 残り時間180秒以内
        self.rem_time = rem_time
        # 自己位置：MyPose.msgの内容：pos: x, y, z, ori_quaternion: x, y, z, w, ori_euler: x, y, z
        # pos
        self.my_pose_pos_list = my_pose_pos_list if my_pose_pos_list != None else [0.0]*3
        """
        # ori_quaternion
        self.my_pose_ori_quaternion_list = [0.0]*4
        """
        # ori_euler
        self.my_pose_ori_euler_list = my_pose_ori_euler_list if my_pose_ori_euler_list != None else [0.0]*3
        # 敵位置
        # pos
        self.enemy_pose_from_camera_pos_list = enemy_pose_from_camera_pos_list if enemy_pose_from_camera_pos_list != None else [0.0]*3
        """
        # ori_quaternion
        self.enemy_pose_from_camera_ori_quaternion_list = [0.0]*4
        """
        # ori_euler
        self.enemy_pose_from_camera_ori_euler_list = enemy_pose_from_camera_ori_euler_list if enemy_pose_from_camera_ori_euler_list != None else [0.0]*3
        

    # 勝ちかどうか
    def is_win(self):
        self_points, enemy_points = calc_game_points(self.score_list)
        if self.rem_time > 0:
            if self_points - enemy_points >= 10:
                return True
        else:
            if self_points - enemy_points > 0:
                return True
        return False

    # 負けかどうか
    def is_lose(self):
        self_points, enemy_points = calc_game_points(self.score_list)
        if self.rem_time > 0:
            if self_points - enemy_points <= -10:
                return True
        else:
            if self_points - enemy_points < 0:
                return True
        return False

    # 引き分けかどうか
    def is_draw(self):
        self_points, enemy_points = calc_game_points(self.score_list)
        return self.rem_time <= 0 and self_points == enemy_points

    # ゲーム終了かどうか
    def is_done(self):
        if self.is_win():
            print('Win game')
        if self.is_lose():
            print('Lose game')
        if self.is_draw():
            print('Draw game')
        return self.is_lose() or self.is_draw() or self.is_win()

    # 合法手のリストの取得
    def legal_actions(self):
        actions = [0,1,2]
        return actions

    # 次の状態更新
    def input_state_array(self):
        self_points, enemy_points = calc_game_points(self.score_list)
        input_state_array = []
        input_state_array.append( [self_points, enemy_points, self.rem_time] )
        input_state_array.append( self.enemy_pos_from_lider_list )
        input_state_array.append( self.my_pose_pos_list )
        input_state_array.append( self.my_pose_ori_euler_list )
        input_state_array.append( self.enemy_pose_from_camera_pos_list )
        input_state_array.append( self.enemy_pose_from_camera_ori_euler_list)
        return input_state_array
    
    def next(self, score, enemy_pos_from_lider_list, rem_time, 
            my_pose_pos_list, my_pose_ori_euler_list, enemy_pose_from_camera_pos_list, enemy_pose_from_camera_ori_euler_list):
            
            return State(score, enemy_pos_from_lider_list, rem_time, 
                                my_pose_pos_list, my_pose_ori_euler_list, enemy_pose_from_camera_pos_list, enemy_pose_from_camera_ori_euler_list)





def calc_game_points(score_list):
    """

    """
    enemy_points = 0
    self_points = 0
    for index, score in enumerate(score_list):
        if score == -1:
            if index == 12 or index == 13:
                enemy_points = enemy_points + 3
            elif index == 14:
                enemy_points = enemy_points + 5
            else:
                enemy_points = enemy_points + 1
            
        elif score == 1:
            if index == 15 or index == 16:
                self_points = self_points + 3
            elif index == 17:
                self_points = self_points + 5
            else:
                self_points = self_points + 1
    return self_points, enemy_points

