cmake_minimum_required(VERSION 2.8.3)
project(pixhawk_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  roscpp
)



## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html


#catkin_install_python(PROGRAMS bin/hello DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS
   bin/Pixhawk_Node
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bin
 )

catkin_package()

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

add_library(PixManagerFunction  src/pix_manager_function.cpp)
target_link_libraries(PixManagerFunction ${catkin_LIBRARIES})

add_executable(PixManager src/pix_manager_node.cpp)
target_link_libraries(PixManager PixManagerFunction ${catkin_LIBRARIES})

catkin_python_setup()
