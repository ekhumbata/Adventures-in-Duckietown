cmake_minimum_required(VERSION 2.8.3)
project(lane_follow)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  std_msgs
  # duckietown_msgs # Every duckietown packages should use this.
)

add_dependencies(${catkin_EXPORTED_TARGETS})

catkin_package(
   CATKIN_DEPENDS
   message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
