cmake_minimum_required(VERSION 2.8.3)
project(save_image)


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
find_package(catkin REQUIRED 
  roscpp
  image_transport
  cv_bridge

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

  ${OpenCV_INCLUDE_DIRS}
)




add_executable(CameraStream src/CameraStream.cpp)
#add_dependencies(CameraStream CameraStream_gencpp ${PROJECT_NAME})
target_link_libraries(CameraStream ${OpenCV_LIBS} ${catkin_LIBRARIES} )


catkin_package()
