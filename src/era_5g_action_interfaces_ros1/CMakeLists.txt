cmake_minimum_required(VERSION 3.0.2)
project(era_5g_action_interfaces_ros1)


find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  std_msgs
  message_generation
  )

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   goal_5g.action
#   Action2.action
 )


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES era_5g_action_interfaces_ros1
 CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

