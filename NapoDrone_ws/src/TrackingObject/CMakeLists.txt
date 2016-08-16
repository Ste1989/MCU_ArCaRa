cmake_minimum_required(VERSION 2.8.3)
project(tracking_object)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  image_transport
  cv_bridge

)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
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
catkin_package(DEPENDS system_lib)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} 

  ${OpenCV_INCLUDE_DIRS}
)

add_executable(TackObject src/main.cpp)
target_link_libraries(TackObject ${OpenCV_LIBS} ${catkin_LIBRARIES} )


catkin_package()
## Declare a C++ library
# add_library(TrackingObject
#   src/${PROJECT_NAME}/TrackingObject.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(TrackingObject ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(TrackingObject_node src/TrackingObject_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(TrackingObject_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(TrackingObject_node
#   ${catkin_LIBRARIES}
# )




