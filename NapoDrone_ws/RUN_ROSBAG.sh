#!/bin/bash

xterm -T "ROSBAG" -n "ROSBAG" -hold -e "rosbag record -O all_topic /aruco_poses /aruco_markers /camera/color/image_raw /mavros/battery /mavros/extended_state /mavros/imu/atm_pressure /mavros/rc/override /mavors/rc/in /mavros/imu/data /mavros/state" &







