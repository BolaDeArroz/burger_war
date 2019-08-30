#!/usr/bin/env python

import sys
import cv2
import roslib
import rospy

from sensor_msgs.msg import LaserScan

class rader_find_enemy:
    def __init__(self):
        #self.image_pub = rospy.Publisher("out_image", Image, queue_size=1)
        #self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/{}/scan'.format(self.name), LaserScan, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        (rows,cols,channels) = cv_image.shape
        cv2.circle(cv_image, (100,100), 100, (0, 0, 255), -1)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)    

def main(args):
    rospy.init_node('image_subscriber', anonymous=True)
    ic = image_subscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)
