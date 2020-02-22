#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import sys
import rospy
import yaml

from PIL import Image

from pyquaternion import Quaternion
#sudo pip install pyquaternion

from nav_msgs.msg import OccupancyGrid

if __name__== '__main__':

    with open("my_map.yaml", "rt") as fp:
        text = fp.read()
    
    map_info = yaml.safe_load(text)

    #read img
    im = np.array(Image.open(map_info['image']), dtype='int8')

    print('##########image info############')
    print('name: %s' % map_info["image"])
    print('size: %s' % str(im.shape))
    print('origin[x,y,yaw]: %s' % str(map_info["origin"]))
    print('resolution: %s' % str(map_info["resolution"]))
    print('################################')

    # set ros node
    publish_gridmap = rospy.Publisher("/map_py", OccupancyGrid, queue_size=10)
    rospy.init_node("map_server_py", anonymous=True)

    # set msg_info
    _map = OccupancyGrid()
    _map.header.stamp = rospy.Time.now()
    _map.header.frame_id = "/red_bot/map"
    _map.info.resolution = map_info["resolution"]
    _map.info.width = im.shape[1]
    _map.info.height = im.shape[0]
    _map.info.origin.position.x = map_info["origin"][0]
    _map.info.origin.position.y = map_info["origin"][1]

    rad = map_info["origin"][2]
    q = Quaternion(axis=[0,0,1], angle=rad).elements
    _map.info.origin.orientation.x = q[1]
    _map.info.origin.orientation.y = q[2]
    _map.info.origin.orientation.z = q[3]
    _map.info.origin.orientation.w = q[0]

    #change occupancy data
    im_py = np.fliplr(im).reshape(-1,1)
    _map.data = im_py

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        publish_gridmap.publish(_map)
        r.sleep()
        print("publish")