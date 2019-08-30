#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from image_function import Normalization, detect_enemy_robot_light


class EnemyFinder():
    def __init__(self):
        self.bridge = CvBridge()

        self.img_pub = rospy.Publisher("enemy_finder", Image, queue_size=1)
        self.atk_pub = rospy.Publisher("array_ex", Float32MultiArray, queue_size=1)
        self.img_sub = rospy.Subscriber("image_raw", Image, self.image_callback)

    def image_callback(self, data):
        image = Normalization(self.bridge.imgmsg_to_cv2(data, "bgr8"))
        enemy = detect_enemy_robot_light(image)
        array = Float32MultiArray()

        h, w, _ = image.shape

        if enemy['red_ball'] != []:
            c, r = cv2.minEnclosingCircle(enemy['red_ball'][0])

            array.data = [1, c[0] - (w / 2.0), 2 * r, w, h]

        elif enemy['green_side'] != []:
            array.data = [-1, 0, 0, w, h]

        else:
            array.data = [0, 0, 0, w, h]

        self.atk_pub.publish(array)
        # self.img_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))


if __name__ == "__main__":
    rospy.init_node("enemy_finder")
    finder = EnemyFinder()
    rospy.spin()
