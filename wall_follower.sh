#!/bin/sh

xterm -e " source ../../devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e " source ../../devel/setup.bash; roslaunch wall_follower gmapping_demo.launch" &
sleep 5
xterm -e " source ../../devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e " source ../../devel/setup.bash; rosrun wall_follower wall_follower" 

