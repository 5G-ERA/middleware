cmake_minimum_required(VERSION 3.0.2)
project(era_5g_action_server_ros1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES era_5g_action_server_ros1
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

# include
  ${catkin_INCLUDE_DIRS}
)


