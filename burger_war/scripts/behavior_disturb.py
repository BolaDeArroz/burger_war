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

from sensor_msgs.msg import Image
from std_msgs.msg import Bool,Float32MultiArray
from geometry_msgs.msg import Point,Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from cv_bridge import CvBridge, CvBridgeError

from image_function import get_tracking_info
from calc_motion_planning import rotation_operate, check_possession_marker

import my_move_base
class bevavior_disturb(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['outcome'])
        #内部のステートマシンsm_subを定義
        #この内部ステートマシンは,outcome
        self.sm_sub = smach.StateMachine(outcomes=['outcome'])
        self.sm_sub.userdata.enemy_pos=Point()
        # sm_subにステートを追加
        # ステートにできるのは、smach.Stateを継承したクラスのみ。(だと思う)
        # smach.StateMachine.addでステートを追加する
        # smach.StateMachine.add(ステート名,クラス名(=実体),
        #                        transitions={'ステートの返り値1':返り値1の時に遷移するステート名,
        #                                     'ステートの返り値2':返り値2の時に遷移するステート名}
        #ってな感じで、遷移先が複数あるならtransitionsをどんどん追加していく
        with self.sm_sub:
            #最初にaddしたステートが開始時のステート
            smach.StateMachine.add('AroundDisturb', AroundDisturb(), 
                                  transitions={'outcome1':'GoEnemyPos', 
                                               'outcome2':'outcome'},
                                  remapping={'enemy_pos_out':'enemy_pos'}) #←sm_sub自体の終了
            smach.StateMachine.add('GoEnemyPos', GoEnemyPos(), 
                                  transitions={'outcome1':'AroundDisturb',
                                               'outcome2':'outcome'},
                                  remapping={'enemy_pos_in':'enemy_pos'}) #←sm_sub自体の終了

        #下2行はsmach_viewerでステートを確認するために必要
        sis = smach_ros.IntrospectionServer('server_name', self.sm_sub, '/SM_Disturb')
        sis.start()
        
    def execute(self,userdata):    
        #内部のステートマシンの実行
        self.sm_sub.execute()
        return 'outcome'



class AroundDisturb(smach.State):
    def enemy_pos_from_lider_callback(self,data):
        self.enemy_pos_from_lider["enemy_pos"]=data
        self.enemy_pos_from_lider["is_topic_receive"]=True

    def enemy_pose_from_camera_callback(self, data):
        self.enemy_pose_from_camera = data

    def enemy_pos_from_score_callback(self,data):
        self.enemy_pos_from_score=data

    def __init__(self):
        #このステートの返り値リストを定義。
        smach.State.__init__(self, outcomes=['outcome1','outcome2'],
                                    output_keys=['enemy_pos_out'])
        
        #停止トピックを受け取るための定義。 
        robot_name=''
        self.name = robot_name
        self.stop_sub = rospy.Subscriber('/{}/state_stop'.format(self.name), Bool, self.stop_callback)
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

        print('+++++++++++++++++++ Disturb +++++++++++++++++++++++++++')
        # rospy終了か、停止トピックを受け取ったらループ抜ける。
        r=rospy.Rate(5)
        self.is_stop_receive=False
        while (not rospy.is_shutdown()) and (self.is_stop_receive==False):
            #敵の位置を推測したらループを抜ける
            #TODO:score情報からの敵位置
            if self.enemy_pos_from_lider["is_topic_receive"]:
                userdata.enemy_pos_out=self.enemy_pos_from_lider["enemy_pos"]
                # TODO
                return 'outcome1'
            else:
                return 'outcome1'
            
            r.sleep()
        
        self.is_stop_receive=False

        return 'outcome2'


    def stop_callback(self,data):
        self.is_stop_receive=True


class GoEnemyPos(smach.State):

    def stop_callback(self,data):
        self.is_stop_receive=True

    def enemy_pos_from_lider_callback(self,data):
        self.enemy_pos_from_lider["enemy_pos"]=data
        self.enemy_pos_from_lider["is_topic_receive"]=True

    def image_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print("image_callback error: ", e)


    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'],
                                    input_keys=['enemy_pos_in'])
        
        #停止トピックを受け取るための定義。 
        robot_name=''
        self.name = robot_name
        self.stop_sub = rospy.Subscriber('/{}/state_stop'.format(self.name), Bool, self.stop_callback)
        self.is_stop_receive=False

        # liderから敵位置推定トピックを受け取るための定義
        self.sub_enemy_pos_from_lider = rospy.Subscriber('/{}/enemy_pos_from_lider'.format(self.name), Point, self.enemy_pos_from_lider_callback)
        self.enemy_pos_from_lider={"enemy_pos":Point(),"is_topic_receive":False}
        # for tracking
        self.image = None
        self.img_sub = rospy.Subscriber("/{}/image_raw".format(self.name), Image, self.image_callback)
        self.bridge = CvBridge()

        #Move base クライアント
        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)


    def execute(self,userdata):
        #パラメータ初期化
        self.enemy_pos_from_lider={"enemy_pos":Point(),"is_topic_receive":False}

        #逃げる位置計算
        enemy_pos=userdata.enemy_pos_in
        
        # my_move_base.setGoal(self.move_base_client,enemy_pos.x,enemy_pos.x,escape_yaw)
        #for cheese burger
        
        escape_yaw= math.pi / 2
        my_move_base.setGoal(self.move_base_client,-0.0,0.65,escape_yaw)
        
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
                return 'outcome2'
            # rotate
            # tracking enemy
            if self.image is not None:
                tracking_info = get_tracking_info(self.image)
                if tracking_info == {}:
                    continue
                if tracking_info['center'][0] > 370:
                    twist = rotation_operate(1)
                elif tracking_info['center'][0] < 270:
                    twist = rotation_operate(2)
                else:
                    twist = rotation_operate(0)
            self.vel_pub.publish(twist)
            #TODO:ゴールが壁の中になった時の対応
            r.sleep()
        #目的地についた場合
        return 'outcome2'
