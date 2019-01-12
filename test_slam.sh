#!/bin/sh

xterm -e " source ../../devel/setup.bash; roslaunch RoboND-HomeService-Project turtlebot_world.launch" &
sleep 5
xterm -e " source ../../devel/setup.bash; roslaunch RoboND-HomeService-Project gmapping_demo.launch" &
sleep 5
xterm -e " source ../../devel/setup.bash; roslaunch RoboND-HomeService-Project view_navigation.launch" &
sleep 5
xterm -e " source ../../devel/setup.bash; roslaunch RoboND-HomeService-Project keyboard_teleop.launch" &

