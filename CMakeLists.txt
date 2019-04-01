cmake_minimum_required(VERSION 2.8.3)
project(proportional_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


################################################
## Declare ROS messages, services and actions ##
################################################

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
)

## Generate messages in the 'msg' folder
add_message_files(FILES
  Goal.msg
)

## Generate services in the 'srv' folder
add_service_files(FILES
  yawControlService.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

##################################
## executables and dependencies ##
##################################

add_executable(
  Controller
  src/Controller.cpp
)
add_dependencies(Controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Controller
  ${catkin_LIBRARIES}
)

add_executable(
  odom_control
  src/odom_control.cpp
)
add_dependencies(odom_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_control
  ${catkin_LIBRARIES}
)

add_executable(
  odom_control_obstacle
  src/odom_control_obstacle.cpp
)

add_dependencies(odom_control_obstacle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  odom_control_obstacle
  ${catkin_LIBRARIES}
)

add_executable(
  odom_control_obstacle_v1
  src/odom_control_obstacle_v1.cpp
)
add_dependencies(
odom_control_obstacle_v1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  odom_control_obstacle_v1
  ${catkin_LIBRARIES}
)

add_executable(
  yaw_control_service
  src/yaw_control_service.cpp
)

add_dependencies(
yaw_control_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  yaw_control_service
  ${catkin_LIBRARIES}
)
