cmake_minimum_required(VERSION 2.8.3)
project(cob_follow_trajectory_interface)

## Add support for C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  rospy
  std_msgs
  actionlib 
  actionlib_msgs
  geometry_msgs
  tf
  moveit_ros_planning
  moveit_ros_planning_interface
)

find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
   CATKIN_DEPENDS control_msgs moveit_core pluginlib roscpp rospy std_msgs actionlib_msgs geometry_msgs tf moveit_ros_planning_interface
   DEPENDS Boost EIGEN3
   INCLUDE_DIRS include
   LIBRARIES cob_follow_trajectory_interface
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(follow_trajectory_utils src/follow_trajectory_utils.cpp)
add_dependencies(follow_trajectory_utils ${catkin_EXPORTED_TARGETS})
target_link_libraries(follow_trajectory_utils ${catkin_LIBRARIES})

add_library(${PROJECT_NAME} src/cob_follow_trajectory_interface.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} follow_trajectory_utils)

add_executable(${PROJECT_NAME}_node src/cob_follow_trajectory_interface_node.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${PROJECT_NAME})

