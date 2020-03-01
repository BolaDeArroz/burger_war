#!/usr/bin/env python
# -*- coding: utf-8 -*-

from navirun import NaviBot
from attack_strategy import AttackStrategy, ATK_STRATEGY_RETIRE_DISTANCE

import rospy


def bola_de_arroz_main():
    navi_bot = NaviBot()
    attack_strategy = AttackStrategy()
    r = rospy.Rate(5) # change speed 5fps
    changed = True
    retire_distance_count = 0
    try:
        while not rospy.is_shutdown():
            # searching enemy
            if changed:
                print('navi')
                navi_bot.strategy()
                changed = False
            else:
                print('attack')
                res = attack_strategy.run(retire_distance_count >= 20)
                changed = True
                retire_distance_count %= 20
                if res == ATK_STRATEGY_RETIRE_DISTANCE:
                    retire_distance_count += 1
                else:
                    retire_distance_count = 0
            r.sleep()
    except KeyboardInterrupt:
        changed = False



if __name__ == '__main__':
    rospy.init_node('bola_de_arroz')
    # main
    bola_de_arroz_main()
    
