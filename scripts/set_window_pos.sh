#!/bin/bash

# Set windows position for Gazebo and JUDGE Window.
# Minimize windows of gnome-terminal(s)

TERMINALS=`xdotool search -onlyvisible gnome-terminal`
for WINID in ${TERMINALS} ; do
 xdotool windowminimize ${WINID}
done

GAZEBOWIN=`xdotool search -onlyvisible Gazebo`
xdotool windowactivate ${GAZEBOWIN}
xdotool windowsize ${GAZEBOWIN} 1000 700
xdotool windowmove ${GAZEBOWIN} 10 10
sleep 0.2
xdotool mousemove 900 70
xdotool click 1
sleep 0.2
xdotool mousemove_relative 50 30 
xdotool click 1
sleep 1.5
xdotool click 1
sleep 1.0
xdotool mousemove 630 380
sleep 0.2
xdotool click 1
sleep 0.2
xdotool click 1
sleep 0.5
xdotool click 4
sleep 0.4
xdotool click 4
sleep 0.4


xdotool windowsize ${GAZEBOWIN} 85% 100%
xdotool windowmove ${GAZEBOWIN} 130 10

JUDGE=`xdotool search -onlyvisible Onigiri`
xdotool windowactivate ${JUDGE}
xdotool windowmove ${JUDGE} 0 10


