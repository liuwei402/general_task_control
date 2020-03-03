cmake_minimum_required(VERSION 2.8.3)
project(general_task_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  device_info_basic.msg
  device_info_task.msg
  device_info_exception.msg
  motion.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate actions in the 'action' folder
add_action_files(
  FILES
  motion_control.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

###################################
## Build ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES general_task_control
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include
)

link_directories(
  ${catkin_LIB_DIRS}
  /usr/lib
  /usr/local/lib
)

## Declare a C++ executable
# general_task_control
add_executable(
  ${PROJECT_NAME}
  src/main.cpp
  src/IotCommunicate.cpp
  src/TaskControl.cpp
  src/MotionControl.cpp
  src/MotionGeneral.cpp
  src/WorkControl.cpp
  src/WorkGeneral.cpp
  src/CurrentPosition.cpp  
)
add_dependencies(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  jsoncpp
)

# send_task_data
add_executable(
  send_task_data
  src/assist/SendTaskData.cpp
)
add_dependencies(
  send_task_data
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  send_task_data
  ${catkin_LIBRARIES}
)

# motion_control_server_test
add_executable(
  motion_control_server_test
  src/assist/MotionControlServerTest.cpp
)
add_dependencies(
  motion_control_server_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  motion_control_server_test
  ${catkin_LIBRARIES}
)