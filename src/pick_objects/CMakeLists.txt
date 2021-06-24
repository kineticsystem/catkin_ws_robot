cmake_minimum_required(VERSION 2.8.3)
project(pick_objects)

set (CMAKE_CXX_FLAGS "-g")

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  NavigationCommand.msg
  OperationStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pick_objects src/pick_objects.cpp)
target_link_libraries(pick_objects ${catkin_LIBRARIES})
