cmake_minimum_required(VERSION 2.8.3)
project(location_manager)


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
   addLocation.srv
   addLocationWithPose.srv
   deleteLocation.srv
   deleteAllLocation.srv
   queryLocation.srv
   queryAllLocation.srv
   queryLocationWithPose.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

 include(FindPkgConfig)
 pkg_check_modules(SQLITE sqlite3)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(locationMgr_server src/locationMgr_service.cpp)
target_link_libraries(locationMgr_server ${catkin_LIBRARIES})
add_dependencies(locationMgr_server ${catkin_EXPORTED_TARGETS})

add_executable(locationMgr_client src/locationMgr_client.cpp)
target_link_libraries(locationMgr_client ${catkin_LIBRARIES})
add_dependencies(locationMgr_client ${catkin_EXPORTED_TARGETS})

target_link_libraries(locationMgr_server /usr/lib/x86_64-linux-gnu/libsqlite3.so)