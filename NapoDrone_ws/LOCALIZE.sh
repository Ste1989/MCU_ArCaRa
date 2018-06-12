#!/bin/bash

ps ax | grep "ros" | awk '{print $1}' | xargs kill -9

sudo chmod 777 /dev/ttyS4

sudo chmod 777 /dev/ttyUSB0

sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/ttyACM1

xterm -T "ROSCORE" -n "ROSCORE" -hold -e "roscore" &

sleep 1

xterm -T "R200" -n "R200" -hold -e "roslaunch realsense_camera r200_autec.launch" &

sleep 1


sleep 5

xterm -T "ARUCO MAPPING" -n "ARUCO MAPPING" -hold -e "roslaunch aruco_mapping aruco_mapping_autec_640x480_2.launch" &


sleep 1

xterm -T "UWB MANAGER" -n "UWB MANAGER" -hold -e "roslaunch uwb_manager UwbManager.launch" &













