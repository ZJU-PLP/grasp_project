cmake_minimum_required(VERSION 2.8.3)
project(grasp_project)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
)
