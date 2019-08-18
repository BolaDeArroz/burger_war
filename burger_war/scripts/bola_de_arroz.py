#!/usr/bin/env python
# -*- coding: utf-8 -*-

from navirun import NaviBot
from attackrun import AttackBot
from attack_strategy import AttackStrategy

import rospy


def bola_de_arroz_main():
    navi_bot = NaviBot()
    attack_strategy = AttackStrategy()
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
                attack_strategy.run(0, 0)
                changed = True
                print('attack')
            r.sleep()
    except KeyboardInterrupt:
        changed = False



if __name__ == '__main__':
    rospy.init_node('bola_de_arroz')
    # main
    bola_de_arroz_main()
    
