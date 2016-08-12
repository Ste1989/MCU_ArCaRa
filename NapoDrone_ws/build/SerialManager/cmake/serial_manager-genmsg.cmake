# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "serial_manager: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iserial_manager:/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/src/SerialManager/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(serial_manager_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/src/SerialManager/msg/Param.msg" NAME_WE)
add_custom_target(_serial_manager_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "serial_manager" "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/src/SerialManager/msg/Param.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(serial_manager
  "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/src/SerialManager/msg/Param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/serial_manager
)

### Generating Services

### Generating Module File
_generate_module_cpp(serial_manager
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/serial_manager
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(serial_manager_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(serial_manager_generate_messages serial_manager_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/src/SerialManager/msg/Param.msg" NAME_WE)
add_dependencies(serial_manager_generate_messages_cpp _serial_manager_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(serial_manager_gencpp)
add_dependencies(serial_manager_gencpp serial_manager_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS serial_manager_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(serial_manager
  "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/src/SerialManager/msg/Param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/serial_manager
)

### Generating Services

### Generating Module File
_generate_module_lisp(serial_manager
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/serial_manager
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(serial_manager_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(serial_manager_generate_messages serial_manager_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/src/SerialManager/msg/Param.msg" NAME_WE)
add_dependencies(serial_manager_generate_messages_lisp _serial_manager_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(serial_manager_genlisp)
add_dependencies(serial_manager_genlisp serial_manager_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS serial_manager_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(serial_manager
  "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/src/SerialManager/msg/Param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/serial_manager
)

### Generating Services

### Generating Module File
_generate_module_py(serial_manager
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/serial_manager
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(serial_manager_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(serial_manager_generate_messages serial_manager_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/src/SerialManager/msg/Param.msg" NAME_WE)
add_dependencies(serial_manager_generate_messages_py _serial_manager_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(serial_manager_genpy)
add_dependencies(serial_manager_genpy serial_manager_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS serial_manager_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/serial_manager)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/serial_manager
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/serial_manager)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/serial_manager
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/serial_manager)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/serial_manager\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/serial_manager
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
