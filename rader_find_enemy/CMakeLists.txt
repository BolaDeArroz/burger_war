cmake_minimum_required(VERSION 2.4.6)
project(rader_find_enemy)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)


catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES rader_fin_enemy
  #CATKIN_DEPENDS roscpp
  #DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


