#!/bin/bash
#set -e
set -x

pkill -f judge

pkill -f setup_sim.launch

pkill -f  sim_robot_run.launch

pkill -f sim_with_judge.sh

pkill -f start.sh

pkill -f gazebo

pkill -f scripts/sim_with_judge.sh

pkill -f scripts/start.sh
