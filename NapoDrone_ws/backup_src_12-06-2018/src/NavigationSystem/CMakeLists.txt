cmake_minimum_required(VERSION 2.8.3)
project(navigation_system)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin  REQUIRED COMPONENTS
  roscpp
  cmake_modules
)
find_package(Eigen REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html


#catkin_install_python(PROGRAMS bin/hello DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


catkin_package(DEPENDS Eigen)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
#NAV SYSTEM
#add_library(NavigationSystemFunction  src/navigation_system_function.cpp)
#target_link_libraries(NavigationSystemFunction ${catkin_LIBRARIES})

#add_executable(NavigationSystem_Node src/navigation_system_node.cpp)
#target_link_libraries(NavigationSystem_Node NavigationSystemFunction ${catkin_LIBRARIES})


#NAV SYSTEM SOLO DECAWAWE
add_library(UWBSystemFunction  src/uwb_system_function.cpp)
target_link_libraries(UWBSystemFunction ${catkin_LIBRARIES})

add_executable(UWBSystem_Node src/uwb_system_node.cpp)
target_link_libraries(UWBSystem_Node UWBSystemFunction ${catkin_LIBRARIES})

#LISTENER PER DECAWAWE
#add_executable(Listener src/listener.cpp)
#target_link_libraries(Listener  ${catkin_LIBRARIES})

#LISTENER PER POZYX
add_executable(Listener_POZ src/listener_poz.cpp)
target_link_libraries(Listener_POZ  ${catkin_LIBRARIES})


