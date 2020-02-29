#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import math
import json
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool,Float32MultiArray,String
import tf


import actionlib
import actionlib_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from image_function import detect_enemy_robot


# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import CameraInfo as CamInfoMSG
import image_geometry
import copy



class NaviBot():
    def __init__(self):
        # bot name 
        # robot_name=''
        robot_name = '' 
        self.name = robot_name
        robot_side=""#rospy.get_param('~side')#TODO launchファイル更新したら書き換え
        if robot_side:
            self.side=robot_side
        else:
            if self.name=='red_bot':self.side='r'
            else:self.side = 'b'
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    
        
        self.is_enemy_detected=False
        self.array=Float32MultiArray()
        rospy.Subscriber("/"+self.name+"/array", Float32MultiArray, self.enemy_detect_callback)

        self.tf_listener=tf.TransformListener()

        #warstate subscriber
        self.war_state = None
        self.war_state_sub = rospy.Subscriber("/"+self.name+"/war_state", String, self.warstate_callback)

        # camera subscribver
        # for convert image topic to opencv obj
        self.image = None
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/{}/image_raw'.format(self.name), Image, self.image_callback)
        self.camerainfo_sub = rospy.Subscriber("/{}/camera_info".format(self.name),CamInfoMSG, self.camerainfo_callback)

    def camerainfo_callback(self,data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

    # camera image call back sample
    # comvert image topic to opencv object and show
    def image_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.original_image = copy.copy(self.image)
        except CvBridgeError as e:
            print("***********************", e)

    def check_possession_marker(self,war_state):
        obtain_targets_list = []
        targets_list = []
        if war_state is None:
            return obtain_targets_list
        #print(dir(war_state.data))
        #print(war_state.data)
        json_war_state = json.loads(war_state.data)
        for key in json_war_state:
            if key == 'scores':
                # TODO
                pass
            elif key == 'targets':
                targets_list = json_war_state[key]
                break
        for target in targets_list:
            for key in target:
                if key == 'player' and target[key] == self.side:
                    obtain_targets_list.append(target['name'])
        return obtain_targets_list

    def enemy_detect_callback(self,array):
        print("[NAVIRUN]EnemyDetect", array.data[0], self.is_enemy_detected, "red size", array.data[2])
        detect_green_result = {}
        if self.image is not None:  
            detect_green_result = detect_enemy_robot(self.image)
        if 'green_side' in detect_green_result and detect_green_result['green_side'] != [] and len(array.data) != 0:
            #print('GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG: ', detect_green_result['green_side'], array)
            self.array.data = [1, 0, 0, 0]
            self.array.data[1]=500
            self.array.data[2]=500
            self.array.data[3]=500
            self.is_enemy_detected = True
        if array.data[2] >= 15 and int(array.data[0]!=0):
            self.is_enemy_detected = True
        self.array=array

    def warstate_callback(self, data):
        self.war_state = data

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

        #wait = self.client.wait_for_result()
        #if not wait:
        #    rospy.logerr("Action server not available!")
        #    rospy.signal_shutdown("Action server not available!")
        #else:
        #    return self.client.get_result()  

    #
    def pub_vel(self,lin_x=0,lin_y=0,ang_z=0,time_sec=0.5):
        cmd_vel=Twist()
        cmd_vel.linear.x=lin_x
        cmd_vel.linear.y=lin_y
        cmd_vel.angular.z=ang_z
        end_time_sec=float(rospy.Time.now().secs+rospy.Time.now().nsecs/(1000*1000*1000)+time_sec)
        while not rospy.is_shutdown():
            rospy.loginfo("[NAVIRUN]cmd_vel.linear.x="+str(cmd_vel.linear.x)+",cmd_vel.linear.y="+str(cmd_vel.linear.y)+",cmd_vel.angular.z="+str(cmd_vel.angular.z))
            self.vel_pub.publish(cmd_vel)
            if (self.is_enemy_detected) or (end_time_sec < float(rospy.Time.now().secs+rospy.Time.now().nsecs/(1000*1000*1000))):
                break
            else:
                rospy.sleep(0.1)
        self.vel_pub.publish(Twist())#stop

    def recovery_abort_behavior(self):   
        print("[NAVIRUN]recovery_abort_behavior")
        #一定距離バック
        self.pub_vel(-0.15,0,0,0.8)

    def swing_behavior(self):
        print("[NAVIRUN]swing_behavior")
        #時計方向に90度回転
        self.pub_vel(0,0,-3.1415/7,1.0)
        #反時計方向に180度回転
        self.pub_vel(0,0,3.1415/7,1.4)
        #時計方向に90度回転
        self.pub_vel(0,0,-3.1415/7,0.7)

    def swing_behavior_right(self):
        print("[NAVIRUN]swing_behavior")
        #時計方向に120度回転
        self.pub_vel(0,0,-3.1415/3.5,2.0)
        #反時計方向に30度回転
        self.pub_vel(0,0,3.1415/3.5,0.3)
        #一時停止
        self.pub_vel(0,0,0,0.5)
        #反時計方向に90度回転
        self.pub_vel(0,0,3.1415/3.5,1.4)

    def calc_nearest_waypoint_idx(self,position,waypoints):
        nearest_waypoint_idx=0
        nearest_distance = 10
        for (idx_waypoint,waypoint) in enumerate(waypoints):
            tmp_distance=math.sqrt((position[0]-waypoint[0])**2 + (position[1]-waypoint[1])**2 )
            if tmp_distance <nearest_distance:
                nearest_distance=tmp_distance
                nearest_waypoint_idx=idx_waypoint
        print("nearest_waypoint_idx1=",nearest_waypoint_idx)
        if nearest_waypoint_idx+1 >= len(waypoints):nearest_waypoint_idx=0
        else:nearest_waypoint_idx=nearest_waypoint_idx+1
        print("nearest_waypoint_idx2=",nearest_waypoint_idx)
        return nearest_waypoint_idx

    def go_waypoint(self,waypoint,is_passing=False):
        r = rospy.Rate(5) # change speed 5fps
        self.setGoal(waypoint[0],waypoint[1],waypoint[2])
        rospy.loginfo("[NAVIRUN]state="+str(self.client.get_state()))
        print("[NAVIRUN]is_enemy_detected",self.is_enemy_detected)
        ret="FAILED"
        while self.client.get_state() in [actionlib_msgs.msg.GoalStatus.ACTIVE,actionlib_msgs.msg.GoalStatus.PENDING] and not self.is_enemy_detected:
            r.sleep()
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(self.name +"/map",self.name +"/base_link", now, rospy.Duration(4.0))

            # map座標系の現在位置をｔｆから取得する
            position, _ = self.tf_listener.lookupTransform(self.name +"/map", self.name +"/base_link", rospy.Time())
            
            # is_passingが有効の時、ウェイポイントのゴールの周囲0.15ｍ以内にロボットが来たら、次のウェイポイントを発行する
            if is_passing and (math.sqrt((position[0]-waypoint[0])**2 + (position[1]-waypoint[1])**2 ) <= 0.15) :
                self.client.cancel_goal()
                ret="SUCCESS"
                break
            
        if self.client.get_state()==actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            ret="SUCCESS"


        if self.client.get_state()==actionlib_msgs.msg.GoalStatus.ABORTED:
            self.recovery_abort_behavior()
            self.client.cancel_goal()
            ret="FAILED"
            
        if self.is_enemy_detected:
            rospy.loginfo("[NAVIRUN]enemy_detected!!")
            self.client.cancel_goal()  
            ret="FAILED"
        
        self.client.cancel_goal()
        print("[NAVIRUN]go_waypoint_result=",ret)
        return ret
    
    def wall_run(self):
        #壁沿い走行時の地点リスト
        wall_run_waypoints = [
            {"pos":[-0.86, 0.0,3.1415/2],"is_swing":True,"is_passing":False},
            {"pos":[-0.81, 0.40,3.1415/4],"is_swing":False,"is_passing":True},
            {"pos":[-0.36, 0.83,       0],"is_swing":False,"is_passing":True},
            {"pos":[-0.00, 0.83,       0],"is_swing":True,"is_passing":False},
            {"pos":[ 0.44, 0.90,-3.1415/4],"is_swing":False,"is_passing":True},
            {"pos":[ 0.85, 0.47,-3.1415/4*2],"is_swing":False,"is_passing":True},
            {"pos":[ 0.85, 0.00,-3.1415/4*2],"is_swing":True,"is_passing":False},
            {"pos":[ 0.94,-0.33,-3.1415/4*3],"is_swing":False,"is_passing":True},
            {"pos":[ 0.45,-0.87,-3.1415],"is_swing":False,"is_passing":True},
            {"pos":[ 0.00,-0.87,-3.1415],"is_swing":True,"is_passing":False},
            {"pos":[-0.46,-0.91,3.1415/4*3],"is_swing":False,"is_passing":True},
            {"pos":[-0.86,-0.47,3.1415/4*2],"is_swing":False,"is_passing":True},
        ]
        #現在位置計算
        cur_position, _ = self.tf_listener.lookupTransform(self.name +"/map", self.name +"/base_link", rospy.Time())
        #cur_eular=tf.transformations.euler_from_quaternion(cur_orientation)
        #最近地点のインデックスを計算
        wall_run_waypoints_pos=[elem["pos"] for elem in wall_run_waypoints]
        nearest_waypoint_idx=self.calc_nearest_waypoint_idx(cur_position,wall_run_waypoints_pos)
        #最近地点からナビゲーション開始
        next_waypoint_idx=nearest_waypoint_idx
        self.is_enemy_detected=False
        while not self.is_enemy_detected:
            waypoint=wall_run_waypoints[next_waypoint_idx]
            rospy.loginfo(str(next_waypoint_idx))
            if self.go_waypoint(waypoint["pos"],is_passing=waypoint["is_passing"]) == "SUCCESS":
                if waypoint["is_swing"]:
                    self.swing_behavior_right()#首を降って索敵
                if next_waypoint_idx+1 >= len(wall_run_waypoints):
                    next_waypoint_idx=0
                else:
                    next_waypoint_idx+=1


    def marker_run(self):
        #フィールドマーカ読み取り時の地点リスト(side=r)
        marker_run_waypoints_r =[
            {"name":u"Tomato_N","pos":[0.77,0.53,3.1415]}, #Tomato_N
            {"name":u"Tomato_S","pos":[0.0,0.53,0.0]},     #Tomato_S
            {"name":u"Omelette_N","pos":[0.77,-0.53,3.1415]},#Omelette_N
            {"name":u"Omelette_S","pos":[0.0,-0.53,3.1415/2]},#Omelette_S
            {"name":u"Pudding_N","pos":[0.0,0.53,3.1415]},  #Pudding_N
            {"name":u"Pudding_S","pos":[-0.77,0.53,0.0]},  #Pudding_S
            {"name":u"OctopusWiener_N","pos":[0.0,-0.53,3.1415]},#OctopusWiener_N
            {"name":u"OctopusWiener_S","pos":[-0.77,-0.53,0.0]},#OctopusWiener_S
            {"name":u"FriedShrimp_N","pos":[0.53,0.0,3.1415]},#FriedShrimp_N
            {"name":u"FriedShrimp_E","pos":[0.0,-0.53,3.1415/2]},#FriedShrimp_E
            {"name":u"FriedShrimp_W","pos":[0.0,0.53,-3.1415/2]},#FriedShrimp_W
            {"name":u"FriedShrimp_S","pos":[-0.53,0.0,0.0]},#FriedShrimp_S
        ]
        #フィールドマーカ読み取り時の地点リスト(side=b)
        marker_run_waypoints_b=[{"name":elem["name"],"pos":[-elem["pos"][0],-elem["pos"][1],elem["pos"][2]+3.1414]} for elem in marker_run_waypoints_r]
        print(marker_run_waypoints_b)
        
        #nameでリスト変更
        if self.name=="red_bot":marker_run_waypoints=marker_run_waypoints_r
        else:marker_run_waypoints=marker_run_waypoints_b

        r=rospy.Rate(5)
        while not self.is_enemy_detected:
            r.sleep()
            obtain_marker_list = self.check_possession_marker(self.war_state)
            print(obtain_marker_list) 

            #まだ得ていないマーカリストを作成
            not_obtain_marker_waypoints=[elem["pos"] for elem in marker_run_waypoints if elem["name"] not in obtain_marker_list]
            not_obtain_marker_names=[elem["name"] for elem in marker_run_waypoints if elem["name"] not in obtain_marker_list]
            print(not_obtain_marker_waypoints)
            print(not_obtain_marker_names)
            if not_obtain_marker_waypoints:#まだ得ていないマーカがある場合
                #現在位置計算
                cur_position, _ = self.tf_listener.lookupTransform(self.name +"/map", self.name +"/base_link", rospy.Time())
                #cur_eular=tf.transformations.euler_from_quaternion(cur_orientation)
                #最近地点のインデックスを計算
                nearest_waypoint_idx=self.calc_nearest_waypoint_idx(cur_position,not_obtain_marker_waypoints)
                waypoint=not_obtain_marker_waypoints[nearest_waypoint_idx]
                print("target_name",not_obtain_marker_names[nearest_waypoint_idx],"target_pos",waypoint)
                #最近地点に移動
                if self.go_waypoint(waypoint,is_passing=False) == "SUCCESS":
                    self.swing_behavior()
            else:#全てのマーカを得た場合
                self.pub_vel(0,0,-3.1415/10,1.0)#回転して索敵    

    def strategy(self,navirun_mode="WALL"):

        try:
            self.tf_listener.waitForTransform(self.name +"/map",self.name +"/base_link",rospy.Time(),rospy.Duration(4.0))
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr("[NAVIRUN]tf_err")
        except Exception as e:
            # I think this error that tf2_ros.TransformException
            print('[NAVIRUN]SSSSSSSSSSSSSSSSSSSs', e)
        self.tf_listener.waitForTransform(self.name +"/map",self.name +"/base_link",rospy.Time(),rospy.Duration(4.0))
        

        ###############################
        #debug code
        #r=rospy.Rate(5)
        #while not rospy.is_shutdown():
        #    r.sleep()
        ###################################
        #navirun_mode="MARKER"
        if navirun_mode=="WALL":#WALL
            self.wall_run()
        else:#MARKER
            self.marker_run()

        return self.array.data[1], self.array.data[2], self.array.data[3]

if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.strategy()


