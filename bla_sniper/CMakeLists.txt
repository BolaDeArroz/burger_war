cmake_minimum_required(VERSION 2.4.6)
project(bla_sniper)

add_compile_options(-std=c++11)

set(OpenCV_DIR /usr/share/OpenCV)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  OpenCV
  message_generation
  sensor_msgs
  )

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES bla_sniper
  CATKIN_DEPENDS roscpp
  DEPENDS OpenCV system_lib)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

add_executable(bla_sniper src/ar_sniper.cpp)
target_link_libraries(bla_sniper
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


