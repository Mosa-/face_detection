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
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES face_detection
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

add_library(fd src/FaceDetection.cpp)
target_link_libraries(fd ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

add_executable(face_detection src/FaceDetectionListener.cpp)

target_link_libraries(face_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  fd
)


