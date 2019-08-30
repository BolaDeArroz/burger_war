#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import math
import numpy as np
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from image_function import Normalization, detect_enemy_robot


class EnemyFinder():
    def __init__(self):
        self.bridge = CvBridge()

        self.img_pub = rospy.Publisher("enemy_finder", Image, queue_size=1)
        self.atk_pub = rospy.Publisher("array", Float32MultiArray, queue_size=1)
        self.img_sub = rospy.Subscriber("image_raw", Image, self.image_callback)

    def image_callback(self, data):
        bgr_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        iim_image = Normalization(bgr_image)

        enemy = detect_enemy_robot(iim_image)

        red_ball_contours = enemy['red_ball']

        array = Float32MultiArray()

        if red_ball_contours != []:
            c, r = cv2.minEnclosingCircle(red_ball_contours[0])

            array.data = [1.0, c[0] - 320, 2 * r, 640, 480]

        else:
            array.data = [0.0, 0, 0, 640, 480]

        self.atk_pub.publish(array)
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(bgr_image, "bgr8")


if __name__ == "__main__":
    rospy.init_node("enemy_finder")
    finder = EnemyFinder()
    rospy.spin()
