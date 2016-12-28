cmake_minimum_required(VERSION 2.8.3)
project(gripper_manager)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

set (mraa_INCLUDE_DIRS "/usr/local/include/mraa")
set (mraa_LIBRARIES "/usr/local/lib/x86_64-linux-gnu/libmraa.so")




## Declare a catkin package
catkin_package()

## Build 
include_directories(include ${catkin_INCLUDE_DIRS}  ${mraa_INCLUDE_DIRS})

add_executable(gripper_manager src/main.cpp)
target_link_libraries(gripper_manager ${catkin_LIBRARIES} ${mraa_LIBRARIES})





