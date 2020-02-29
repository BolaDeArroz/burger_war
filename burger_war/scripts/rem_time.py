#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import Time


class RemTime:
    def __init__(self):
        self.pub = rospy.Publisher(NODE_NAME, Time, queue_size=1)

        self.time = int(rospy.get_param("~time"))
        self.unit = int(rospy.get_param("~unit"))

        rospy.Timer(rospy.Duration(self.unit), self.timer_callback)

    def timer_callback(self, data):
        self.time -= self.unit

        if self.time < 0:
            self.time = 0

        msg = Time()

        msg.data = rospy.Time(secs=self.time)

        self.pub.publish(msg)


NODE_NAME = "rem_time"


if __name__ == "__main__":
    rospy.init_node(NODE_NAME)

    node = RemTime()

    rospy.spin()
