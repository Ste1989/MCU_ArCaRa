#!/bin/bash

ps ax | grep "ros" | awk '{print $1}' | xargs kill -9

sudo chmod 777 /dev/ttyS4

sudo chmod 777 /dev/ttyUSB0

xterm -T "ROSCORE" -n "ROSCORE" -hold -e "roscore" &

sleep 1

xterm -T "R200" -n "R200" -hold -e "roslaunch realsense_camera r200_autec.launch" &

sleep 1

xterm -T "ARUCO MAPPING" -n "ARUCO MAPPING" -hold -e "roslaunch aruco_mapping aruco_mapping_autec_640x480.launch" &

sleep 2

xterm -T "SERIAL MANAGER" -n "SERIAL MANAGER" -hold -e "roslaunch serial_manager SerialManager.launch" &

sleep 1

xterm -T "AUTOPILOT MANAGER" -n "AUTOPILOT MANAGER" -hold -e "roslaunch autopilot_manager AutopilotManagerMavros.launch" &

sleep 5

xterm -T "MAVROS" -n "MAVROS" -hold -e "roslaunch mavros dji550_autec.launch" &

sleep 30

xterm -T "MAVROS SYSID" -n "MAVROS SYSID" -hold -e "rosrun mavros mavparam set SYSID_MYGCS 1" &










