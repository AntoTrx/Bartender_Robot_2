# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "move_arm: 0 messages, 1 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Iintera_core_msgs:/opt/ros/eecsbot_ws/src/intera_common/intera_core_msgs/msg;-Iintera_core_msgs:/opt/ros/eecsbot_ws/devel/share/intera_core_msgs/msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(move_arm_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cc/ee106a/fa22/class/ee106a-aee/ros_workspaces/bartender_robot/src/move_arm/srv/target.srv" NAME_WE)
add_custom_target(_move_arm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "move_arm" "/home/cc/ee106a/fa22/class/ee106a-aee/ros_workspaces/bartender_robot/src/move_arm/srv/target.srv" "geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/Pose"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(move_arm
  "/home/cc/ee106a/fa22/class/ee106a-aee/ros_workspaces/bartender_robot/src/move_arm/srv/target.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move_arm
)

### Generating Module File
_generate_module_cpp(move_arm
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move_arm
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(move_arm_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(move_arm_generate_messages move_arm_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa22/class/ee106a-aee/ros_workspaces/bartender_robot/src/move_arm/srv/target.srv" NAME_WE)
add_dependencies(move_arm_generate_messages_cpp _move_arm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_arm_gencpp)
add_dependencies(move_arm_gencpp move_arm_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_arm_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(move_arm
  "/home/cc/ee106a/fa22/class/ee106a-aee/ros_workspaces/bartender_robot/src/move_arm/srv/target.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move_arm
)

### Generating Module File
_generate_module_eus(move_arm
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move_arm
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(move_arm_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(move_arm_generate_messages move_arm_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa22/class/ee106a-aee/ros_workspaces/bartender_robot/src/move_arm/srv/target.srv" NAME_WE)
add_dependencies(move_arm_generate_messages_eus _move_arm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_arm_geneus)
add_dependencies(move_arm_geneus move_arm_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_arm_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(move_arm
  "/home/cc/ee106a/fa22/class/ee106a-aee/ros_workspaces/bartender_robot/src/move_arm/srv/target.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move_arm
)

### Generating Module File
_generate_module_lisp(move_arm
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move_arm
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(move_arm_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(move_arm_generate_messages move_arm_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa22/class/ee106a-aee/ros_workspaces/bartender_robot/src/move_arm/srv/target.srv" NAME_WE)
add_dependencies(move_arm_generate_messages_lisp _move_arm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_arm_genlisp)
add_dependencies(move_arm_genlisp move_arm_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_arm_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(move_arm
  "/home/cc/ee106a/fa22/class/ee106a-aee/ros_workspaces/bartender_robot/src/move_arm/srv/target.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move_arm
)

### Generating Module File
_generate_module_nodejs(move_arm
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move_arm
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(move_arm_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(move_arm_generate_messages move_arm_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa22/class/ee106a-aee/ros_workspaces/bartender_robot/src/move_arm/srv/target.srv" NAME_WE)
add_dependencies(move_arm_generate_messages_nodejs _move_arm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_arm_gennodejs)
add_dependencies(move_arm_gennodejs move_arm_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_arm_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(move_arm
  "/home/cc/ee106a/fa22/class/ee106a-aee/ros_workspaces/bartender_robot/src/move_arm/srv/target.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_arm
)

### Generating Module File
_generate_module_py(move_arm
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_arm
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(move_arm_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(move_arm_generate_messages move_arm_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa22/class/ee106a-aee/ros_workspaces/bartender_robot/src/move_arm/srv/target.srv" NAME_WE)
add_dependencies(move_arm_generate_messages_py _move_arm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_arm_genpy)
add_dependencies(move_arm_genpy move_arm_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_arm_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move_arm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move_arm
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(move_arm_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET intera_core_msgs_generate_messages_cpp)
  add_dependencies(move_arm_generate_messages_cpp intera_core_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(move_arm_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(move_arm_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move_arm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move_arm
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(move_arm_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET intera_core_msgs_generate_messages_eus)
  add_dependencies(move_arm_generate_messages_eus intera_core_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(move_arm_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(move_arm_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move_arm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move_arm
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(move_arm_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET intera_core_msgs_generate_messages_lisp)
  add_dependencies(move_arm_generate_messages_lisp intera_core_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(move_arm_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(move_arm_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move_arm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move_arm
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(move_arm_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET intera_core_msgs_generate_messages_nodejs)
  add_dependencies(move_arm_generate_messages_nodejs intera_core_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(move_arm_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(move_arm_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_arm)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_arm\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_arm
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(move_arm_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET intera_core_msgs_generate_messages_py)
  add_dependencies(move_arm_generate_messages_py intera_core_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(move_arm_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(move_arm_generate_messages_py std_msgs_generate_messages_py)
endif()
