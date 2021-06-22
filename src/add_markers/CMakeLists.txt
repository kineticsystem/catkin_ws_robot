cmake_minimum_required(VERSION 2.8.3)
project(add_markers)

set (CMAKE_CXX_FLAGS "-g")

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_arm
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(add_markers src/add_markers.cpp)
target_link_libraries(add_markers ${catkin_LIBRARIES})
