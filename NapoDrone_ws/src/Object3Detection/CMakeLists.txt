cmake_minimum_required(VERSION 2.8.3)
project(obj_3d_detection)


# set a default build type if none was specified
#IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  #MESSAGE(STATUS "Setting build type to 'Release' as none was specified.")
 # SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  #SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
  #  "MinSizeRel" "RelWithDebInfo")
#ENDIF()

# default build is in Release mode
#IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release")
#ENDIF()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED COMPONENTS roscpp)

set(PCL_DIR "/usr/share/pcl-1.7/")
find_package(catkin REQUIRED 
  roscpp
  image_transport
  cv_bridge
  PCL
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)

find_package( OpenCV  REQUIRED)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  DEPENDS system_lib)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_library(Vision3DFunction  src/3Dvision_function.cpp)
target_link_libraries(Vision3DFunction  ${OpenCV_LIBS} ${PCL_LIBRARIES})

#compilare eseguibile 
add_executable(Vision3DNode src/3Dvision_node.cpp)
target_link_libraries(Vision3DNode Vision3DFunction ${OpenCV_LIBS} ${catkin_LIBRARIES} )



catkin_package()
