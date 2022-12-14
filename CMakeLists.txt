cmake_minimum_required(VERSION 3.0.2)
project(cartesian_trajectory_generator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    actionlib_msgs
    dynamic_reconfigure
    eigen_conversions
    geometry_msgs
    interactive_markers
    message_generation
    roscpp
    tf
    tf_conversions
    visualization_msgs
)

find_package(Eigen3 REQUIRED)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Trajectory.action
)

add_message_files(
  FILES
  OverlayMotionConf.msg
)

add_service_files(
  FILES
  OverlayMotion.srv
)

generate_dynamic_reconfigure_options(
  cfg/pose_param.cfg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES cartesian_trajectory_generator
  CATKIN_DEPENDS actionlib_msgs dynamic_reconfigure eigen_conversions geometry_msgs interactive_markers message_runtime roscpp tf tf_conversions visualization_msgs
  DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/cartesian_trajectory_generator_ros_node.cpp src/cartesian_trajectory_generator_ros.cpp)

add_dependencies(
  ${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${ros_logger_LIBRARIES}
)
