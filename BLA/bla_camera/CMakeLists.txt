cmake_minimum_required(VERSION 2.4.6)
project(bla_camera)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  OpenCV
  message_generation
  )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bla_camera
  CATKIN_DEPENDS roscpp
  DEPENDS opencv system_lib)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(camera_subscriber src/camera_subscriber.cpp)
target_link_libraries(camera_subscriber
  ${catkin_LIBRARIES}
)


