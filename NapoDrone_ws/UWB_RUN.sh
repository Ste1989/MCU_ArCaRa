#!/bin/bash

ps ax | grep "ros" | awk '{print $1}' | xargs kill -9



sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyACM0


xterm -T "ROSCORE" -n "ROSCORE" -hold -e "roscore" &

sleep 1

xterm -T "UWB" -n "UWB" -hold -e "rosrun pozyxmanager pose_pub.py" &

sleep 1


sleep 5

xterm -T "NAV" -n "NAV" -hold -e "roslaunch navigation_system UWBSystem.launch" &


sleep 1















