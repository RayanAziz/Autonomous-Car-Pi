cmake_minimum_required(VERSION 2.8)
project( opencv )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
find_package(catkin REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})

add_executable( opencv HOG_Deploy.cpp )
target_link_libraries( opencv ${OpenCV_LIBS} )
target_link_libraries( opencv ${catkin_LIBRARIES} )
