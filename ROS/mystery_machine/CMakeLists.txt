cmake_minimum_required(VERSION 2.8.3)
project(mystery_machine)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(forebrain src/forebrain.cpp)
target_link_libraries(forebrain ${catkin_LIBRARIES})
