#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class BallFinder():
    def __init__(self):
        rospy.init_node("ball_finder")

        self.bridge = CvBridge()

        self.img_pub = rospy.Publisher("attack_finder", Image, queue_size=1)
        self.atk_pub = rospy.Publisher("move_x", Bool, queue_size=1)
        self.img_sub = rospy.Subscriber("image_raw", Image, self.image_callback)

    def image_callback(self, data):
	bgr_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        mask = make_red_mask(bgr_image)

        ball = find_ball(mask)

        if ball is not None:
            self.atk_pub.publish(True)

            self.output_debug(bgr_image, mask, ball)

        else:
            self.atk_pub.publish(False)

    def output_debug(self, bgr_image, mask, ball):
        masked = cv2.bitwise_and(bgr_image, bgr_image, mask=mask)

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(masked, "bgr8"))

        print("[BallFinder]", ball[0], ball[1])


def make_red_mask(bgr_image):
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV_FULL)

    h = hsv_image[:, :, 0]
    s = hsv_image[:, :, 1]

    mask = np.zeros(h.shape, dtype=np.uint8)

    mask[((h < 20) | (h > 200)) & (s > 150)] = 255

    return mask


def find_ball(bin_image):
    _, cols = bin_image.shape

    dots = np.count_nonzero(bin_image != 0)

    if dots == 0:
        return None

    m = cv2.moments(bin_image)

    x = int(cols / 2) - int(m["m10"] / m["m00"])

    return dots, x


if __name__ == "__main__":
    finder = BallFinder()

    rospy.spin()
