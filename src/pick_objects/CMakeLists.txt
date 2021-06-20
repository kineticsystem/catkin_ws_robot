cmake_minimum_required(VERSION 2.8.3)
project(pick_objects)

set (CMAKE_CXX_FLAGS "-g")

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    actionlib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_arm
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pick_objects src/pick_objects.cpp)
target_link_libraries(pick_objects ${catkin_LIBRARIES})
