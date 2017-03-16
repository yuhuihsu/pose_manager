cmake_minimum_required(VERSION 2.8.3)
project(pose_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  image_geometry
  pose_manager
)

add_message_files(
   FILES
   Location.msg
   LocationArray.msg
)

 add_service_files(
   FILES
   add.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
