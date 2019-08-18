#!/usr/bin/env python
# -*- coding: utf-8 -*-

from navirun import NaviBot
from attackrun import AttackBot

import rospy


def bola_de_arroz_main():
    navi_bot = NaviBot()
    attack_bot = AttackBot()
    r = rospy.Rate(5) # change speed 5fps
    changed = True
    try:
        while not rospy.is_shutdown():
            # searching enemy
            if changed:   
                navi_bot.strategy()  
                print('navi')
                changed = False
            else:
                attack_bot.attack_war()
                changed = True
                print('attack')
            r.sleep()
    except KeyboardInterrupt:
        changed = False



if __name__ == '__main__':
    rospy.init_node('bola_de_arroz')
    # main
    bola_de_arroz_main()
    