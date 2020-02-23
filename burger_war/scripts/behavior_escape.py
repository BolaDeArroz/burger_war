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
from geometry_msgs.msg import Point

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class bevavior_escape(smach.State):
    def __init__(self):
        #robot name 
        robot_name=rospy.get_param('~robot_name')
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

        robot_name=rospy.get_param('~robot_name')
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
        while (not rospy.is_shutdown()) or (self.is_stop_receive==False):
            #敵の位置を推測
            if self.enemy_pos_from_lider["is_topic_receive"]:
                userdata.enemy_pos_out=self.enemy_pos_from_lider["enemy_pos"]
                return 'is_EnemyFound'
        
        self.is_stop_receive=False
        return 'is_receiveStopSig'


class GoToEscapePoint(smach.State):

    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.name + "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
 
        self.client.send_goal(goal)

        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()  


    def stop_callback(self,data):
        self.is_stop_receive=True


    def __init__(self):
        smach.State.__init__(self,  outcomes=['is_Gone','is_receiveStopSig'],
                                    input_keys=['enemy_pos_in'])

        robot_name=rospy.get_param('~robot_name')
        self.name = robot_name

        #停止トピックを受け取るための定義。         
        self.sub_stop = rospy.Subscriber('/{}/state_stop'.format(self.name), Bool, self.stop_callback)
        self.is_stop_receive=False

        #Move base クライアント
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)


    def execute(self,userdata):
        r=rospy.Rate(5)
        # rospy終了か、停止トピックを受け取ったらループ抜ける。
        self.is_stop_receive=False
        while (not rospy.is_shutdown()) or (self.is_stop_receive==False):
            #逃げる位置計算
            enemy_pos=userdata.enemy_pos_in
            print(enemy_pos)
            #中心から見て反対地点に逃げる
            escape_pos_x=enemy_pos.x
            escape_pos_y=enemy_pos.y
            

            #逃げる処理
            self.setGoal(-escape_pos_y,escape_pos_x,0)

            return 'is_Gone'
        
        self.is_stop_receive=False
        return 'is_receiveStopSig'

