cmake_minimum_required(VERSION 2.8.3)
project(uwb_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp  message_generation)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} 
  ${SDFormat_INCLUDE_DIRS}
)







add_library(UWBManagerFunction  src/uwb_manager_function.cpp)
target_link_libraries(UWBManagerFunction   ${catkin_LIBRARIES})

add_executable(UWBManager src/uwb_manager_node.cpp)
target_link_libraries(UWBManager UWBManagerFunction ${catkin_LIBRARIES} )




catkin_package( 
	DEPENDS system_lib
	
	)

