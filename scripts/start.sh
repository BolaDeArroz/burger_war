#!/bin/bash

# set judge server state "running"
bash judge/test_scripts/set_running.sh localhost:5000

# launch robot control node
# roslaunch burger_war sim_robot_run.launch
# roslaunch burger_war sim_level_1_cheese.launch
roslaunch burger_war sim_level_2_teriyaki.launch