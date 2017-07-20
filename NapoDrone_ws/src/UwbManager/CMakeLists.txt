cmake_minimum_required(VERSION 2.8.3)
project(uwb_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
)



## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html


#catkin_install_python(PROGRAMS bin/hello DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_message_files(
  FILES
  RangeUwb.msg 
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(	DEPENDS system_lib
	CATKIN_DEPENDS message_runtime)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(UwbManagerFunction  src/uwb_manager_function.cpp)
target_link_libraries(UwbManagerFunction ${catkin_LIBRARIES})

add_executable(UwbManager_Node src/uwb_manager_node.cpp)
target_link_libraries(UwbManager_Node UwbManagerFunction ${catkin_LIBRARIES})


