# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "utilities: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iutilities:/home/robocuphome/robocuphome2015/src/utilities/msg;-Istd_msgs:/opt/ros/groovy/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(utilities_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(utilities
  "/home/robocuphome/robocuphome2015/src/utilities/msg/gripperTestStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/utilities
)
_generate_msg_cpp(utilities
  "/home/robocuphome/robocuphome2015/src/utilities/msg/gripperTestRequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/utilities
)

### Generating Services

### Generating Module File
_generate_module_cpp(utilities
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/utilities
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(utilities_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(utilities_generate_messages utilities_generate_messages_cpp)

# target for backward compatibility
add_custom_target(utilities_gencpp)
add_dependencies(utilities_gencpp utilities_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS utilities_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(utilities
  "/home/robocuphome/robocuphome2015/src/utilities/msg/gripperTestStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/utilities
)
_generate_msg_lisp(utilities
  "/home/robocuphome/robocuphome2015/src/utilities/msg/gripperTestRequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/utilities
)

### Generating Services

### Generating Module File
_generate_module_lisp(utilities
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/utilities
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(utilities_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(utilities_generate_messages utilities_generate_messages_lisp)

# target for backward compatibility
add_custom_target(utilities_genlisp)
add_dependencies(utilities_genlisp utilities_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS utilities_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(utilities
  "/home/robocuphome/robocuphome2015/src/utilities/msg/gripperTestStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/utilities
)
_generate_msg_py(utilities
  "/home/robocuphome/robocuphome2015/src/utilities/msg/gripperTestRequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/utilities
)

### Generating Services

### Generating Module File
_generate_module_py(utilities
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/utilities
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(utilities_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(utilities_generate_messages utilities_generate_messages_py)

# target for backward compatibility
add_custom_target(utilities_genpy)
add_dependencies(utilities_genpy utilities_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS utilities_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/utilities)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/utilities
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(utilities_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/utilities)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/utilities
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(utilities_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/utilities)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/utilities\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/utilities
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(utilities_generate_messages_py std_msgs_generate_messages_py)
