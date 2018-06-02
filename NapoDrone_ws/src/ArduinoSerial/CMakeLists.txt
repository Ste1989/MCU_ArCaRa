cmake_minimum_required(VERSION 2.8.3)
project(arduino_serial)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp  )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package( 
	DEPENDS system_lib
	
	)


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








add_library(ArduinoSerialFunction  src/arduino_serial_function.cpp)
target_link_libraries(ArduinoSerialFunction  ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(ArduinoSerial src/arduino_serial_node.cpp)
target_link_libraries(ArduinoSerial ArduinoSerialFunction ${OpenCV_LIBS} ${catkin_LIBRARIES} )






