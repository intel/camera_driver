cmake_minimum_required(VERSION 2.8.3)
project(camera_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  camera_info_manager
)

set(CMAKE_CXX_FLAGS "-std=c++11")
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_driver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

option(DEFINE_DEBUG
  "Build the project with debug logs enabled"
  OFF)
if(DEFINE_DEBUG)
  add_definitions(-DDEBUG)
endif()

add_executable(${PROJECT_NAME}_node src/camera_driver_node.cpp src/CameraDeviceAtomIsp.cpp src/v4l2_interface.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

