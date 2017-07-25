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

xterm -T "AUTOPILOT MANAGER" -n "AUTOPILOT MANAGER" -hold -e "roslaunch autopilot_manager AutopilotManagerMavros.launch" &

sleep 5

xterm -T "ARUCO MAPPING" -n "ARUCO MAPPING" -hold -e "roslaunch aruco_mapping aruco_mapping_autec_640x480_2.launch" &

sleep 2

xterm -T "SERIAL MANAGER" -n "SERIAL MANAGER" -hold -e "roslaunch serial_manager SerialManager.launch" &

sleep 1

xterm -T "GRIPPER MANAGER" -n "GRIPPER MANAGER" -hold -e "roslaunch buzzer_manager buzzer.launch" &

sleep 1

xterm -T "BUZZER MANAGER" -n "BUZZER MANAGER" -hold -e "roslaunch gripper_manager gripper.launch" &

sleep 1

xterm -T "NAVIGATION SYSTEM" -n "NAVIGATION SYSTEM" -hold -e "roslaunch navigation_system NavigationSystem.launch" &

sleep 1

xterm -T "UWB MANAGER" -n "UWB MANAGER" -hold -e "roslaunch uwb_manager UwbManager.launch" &

sleep 1

xterm -T "MAVROS" -n "MAVROS" -hold -e "roslaunch mavros dji550_autec.launch" &

sleep 30

xterm -T "MAVROS SYSID" -n "MAVROS SYSID" -hold -e "rosrun mavros mavparam set SYSID_MYGCS 1" &










