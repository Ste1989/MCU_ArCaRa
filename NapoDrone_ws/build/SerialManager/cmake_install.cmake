# Install script for directory: /home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/src/SerialManager

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/serial_manager/msg" TYPE FILE FILES "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/src/SerialManager/msg/Param.msg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/serial_manager/cmake" TYPE FILE FILES "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/build/SerialManager/catkin_generated/installspace/serial_manager-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/devel/include/serial_manager")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/devel/share/common-lisp/ros/serial_manager")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/devel/lib/python2.7/dist-packages/serial_manager")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/devel/lib/python2.7/dist-packages/serial_manager")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/build/SerialManager/catkin_generated/installspace/serial_manager.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/serial_manager/cmake" TYPE FILE FILES "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/build/SerialManager/catkin_generated/installspace/serial_manager-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/serial_manager/cmake" TYPE FILE FILES
    "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/build/SerialManager/catkin_generated/installspace/serial_managerConfig.cmake"
    "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/build/SerialManager/catkin_generated/installspace/serial_managerConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/serial_manager" TYPE FILE FILES "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/src/SerialManager/package.xml")
endif()

