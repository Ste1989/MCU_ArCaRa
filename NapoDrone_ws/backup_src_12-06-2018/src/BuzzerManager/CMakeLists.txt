cmake_minimum_required(VERSION 2.8.3)
project(buzzer_manager)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

set (mraa_INCLUDE_DIRS "/usr/local/include/mraa")
set (mraa_LIBRARIES "/usr/local/lib/x86_64-linux-gnu/libmraa.so")




## Declare a catkin package
catkin_package()

## Build 
include_directories(include ${catkin_INCLUDE_DIRS}  ${mraa_INCLUDE_DIRS})

add_library(BuzzerFunction  src/buzzer_function.cpp)
target_link_libraries(BuzzerFunction  ${catkin_LIBRARIES} ${mraa_LIBRARIES})

add_executable(buzzer_manager src/main.cpp)
target_link_libraries(buzzer_manager BuzzerFunction ${catkin_LIBRARIES} ${mraa_LIBRARIES})





