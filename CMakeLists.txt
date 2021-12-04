cmake_minimum_required(VERSION 3.0.2)
project(mission_planner)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    darknet_ros_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    darknet_ros_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
