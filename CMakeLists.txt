cmake_minimum_required(VERSION 2.8.3)
project(face_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  OpenCV
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES face_detection
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

add_executable(face_detection src/FaceDetection.cpp)

target_link_libraries(face_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


