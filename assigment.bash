#!/bin/bash 
	
echo " Launching assignment package"
sleep 4

echo " Launching iris Gazebo"
cd ~/PX4-Autopilot
xterm -e "HEADLESS=1 make px4_sitl gazebo" &

sleep 2
xterm -e roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"  &
sleep 2

sleep 4
echo " Running ROS node"

sleep 2
cd ~/Desktop
xterm -e ./QGroundControl.AppImage   &


sleep 6
xterm -e python3 
