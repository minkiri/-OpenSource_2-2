cmake_minimum_required(VERSION 3.0.2)
project(Marker_Tracking)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
        cv_bridge
        image_transport
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES loca_path
  CATKIN_DEPENDS ackermann_msgs geometry_msgs nav_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
        ${Opencv_INCLUDE_DIRS}

)

add_executable(${PROJECT_NAME}
 include/local_path.h
 include/pure_pursuit.h
 include/ros_data.h
 
 src/local_path.cpp
 src/pure_pursuit.cpp
        src/ros_data.cpp
        include/PID.h)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

