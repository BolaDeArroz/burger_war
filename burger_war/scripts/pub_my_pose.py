#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import math
import roslib
import rospy
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3
from burger_war.msg import MyPose 
import tf
class pub_my_pose:
    def __init__(self):
        # bot name 
        robot_name=rospy.get_param('~robot_name')
        self.name = robot_name

        self.tf_listener=tf.TransformListener()
        try:
            self.tf_listener.waitForTransform(self.name +"/map",self.name +"/base_link",rospy.Time(),rospy.Duration(4.0))
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr("[rader_find_enemy]tf_err")
        except Exception as e:
            # I think this error that tf2_ros.TransformException
            print('[rader_find_enemy]', e)
        self.tf_listener.waitForTransform(self.name +"/map",self.name +"/base_link",rospy.Time(),rospy.Duration(4.0))

        self.my_pose_publisher=rospy.Publisher('my_pose',MyPose,queue_size=1)

    def run(self):
        r=rospy.Rate(5)
        while not rospy.is_shutdown():
            #map座標系上のロボット現在位置(my_pos)・姿勢(my_ori)をtfから取得する
            my_pos, my_ori = self.tf_listener.lookupTransform(self.name +"/map", self.name +"/base_link",rospy.Time())
            #姿勢(my_ori)がQuaternion形式なのでEuler形式(my_eul)に変換する
            my_eul=tf.transformations.euler_from_quaternion(my_ori)
            #my_posはxが横軸、yが縦軸になるように並び替える。
            #       xはロボットスタート向きから、  右方向が+
            #       yはロボットスタート向きから、正面方向が+
            #       zは0
            #my_oriは取得したそのまま。
            #my_eulは  右方向が0,上がπ/2,左がπ, (or -π)下が-π/2になるように調整  
            my_theta=my_eul[2]+math.pi/2 if my_eul[2]+math.pi/2 < math.pi else my_eul[2]-math.pi*3/2
            my_pose=MyPose(pos           =Point(     -my_pos[1], my_pos[0],         0          ),\
                           ori_quaternion=Quaternion( my_ori[0], my_ori[1], my_ori[2],my_ori[3]),\
                           ori_euler     =Vector3(            0,         0, my_theta)          )
            self.my_pose_publisher.publish(my_pose)
            r.sleep()

        
def main(args):
    rospy.init_node('my_pose', anonymous=True)
    ra = pub_my_pose()
    print('[pub_my_pose]initialized')
    ra.run()

if __name__=='__main__':
    main(sys.argv)
