#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import Time


class RemTime:
    def __init__(self):
        self.__pub = rospy.Publisher(NODE_NAME, Time, queue_size=1)

        self.__time = int(rospy.get_param("~time"))
        self.__unit = int(rospy.get_param("~unit"))

        rospy.Timer(rospy.Duration(self.__unit), self.__timer_callback)

    def __timer_callback(self, data):
        self.__time -= self.__unit

        if self.__time < 0:
            self.__time = 0

        msg = Time()

        msg.data = rospy.Time(secs=self.__time)

        self.__pub.publish(msg)


NODE_NAME = "rem_time"


if __name__ == "__main__":
    rospy.init_node(NODE_NAME)

    node = RemTime()

    rospy.spin()
