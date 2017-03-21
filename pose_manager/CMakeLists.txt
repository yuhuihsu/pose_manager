cmake_minimum_required(VERSION 2.8.3)
project(pose_manager)
link_directories(/usr/lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  image_geometry
)

add_message_files(
   FILES
   Location.msg
   LocationArray.msg
)

add_service_files(
   FILES
   insert.srv
   update.srv
   deleteName.srv
   deletePose.srv
   deleteAll.srv
   queryPose.srv
   queryName.srv
   queryAll.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)
 # enable pkg_check_modules macro
 include(FindPkgConfig)
 pkg_check_modules(SQLITE sqlite3)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(poseMgr_server src/poseMgr_service.cpp)
target_link_libraries(poseMgr_server ${catkin_LIBRARIES})
add_dependencies(poseMgr_server ${catkin_EXPORTED_TARGETS})

add_executable(poseMgr_client src/poseMgr_client.cpp)
target_link_libraries(poseMgr_client ${catkin_LIBRARIES})
add_dependencies(poseMgr_client ${catkin_EXPORTED_TARGETS})

target_link_libraries(poseMgr_server /usr/lib/x86_64-linux-gnu/libsqlite3.so)