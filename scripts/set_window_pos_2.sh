#!/bin/bash

# Set Window size and positions for RVIZ

DESKTOPWIDTH=`xdpyinfo  | grep -oP 'dimensions:\s+\K\S+' | cut -f 1 -d "x"`
DESKTOPHEIGHT=`xdpyinfo  | grep -oP 'dimensions:\s+\K\S+' | cut -f 2 -d "x"`

RVIZSIZE=400

RVIZWIN1=`xdotool search -onlyvisible ".*red.*Rviz"`

if [ -n $RVIZWIN1 ]; then
xdotool windowactivate ${RVIZWIN1}
xdotool windowmove ${RVIZWIN1} 0 10
xdotool mousemove 377 500
sleep 0.2
xdotool click 1
sleep 0.2
xdotool windowsize ${RVIZWIN1} ${RVIZSIZE} ${RVIZSIZE}
xdotool windowmove ${RVIZWIN1} `expr ${DESKTOPWIDTH} - ${RVIZSIZE}` 10
fi

RVIZWIN2=`xdotool search -onlyvisible ".*blue.*Rviz"`

if [ -n $RVIZWIN2 ]; then
xdotool windowactivate ${RVIZWIN2}
xdotool windowmove ${RVIZWIN2} 0 10
xdotool mousemove 377 500
sleep 0.2
xdotool click 1
sleep 0.2
xdotool windowsize ${RVIZWIN2} ${RVIZSIZE} ${RVIZSIZE}
xdotool windowmove ${RVIZWIN2} 10 320
fi

GAZEBOWIN=`xdotool search -onlyvisible Gazebo`
xdotool windowactivate ${GAZEBOWIN}

JUDGE=`xdotool search -onlyvisible Onigiri`
xdotool windowactivate ${JUDGE}

if [ -n $RVIZWIN1 ]; then
xdotool windowactivate ${RVIZWIN1}
fi
if [ -n $RVIZWIN2 ]; then
xdotool windowactivate ${RVIZWIN2}
fi
TERMINALS=`xdotool search -onlyvisible gnome-terminal`
for WINID in ${TERMINALS} ; do
 xdotool windowminimize ${WINID}
done
