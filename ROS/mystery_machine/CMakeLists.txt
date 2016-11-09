cmake_minimum_required(VERSION 2.8.3)
project(mystery_machine)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talk src/talker.cpp)
target_link_libraries(talk ${catkin_LIBRARIES})

add_executable(listen src/listener.cpp)
target_link_libraries(listen ${catkin_LIBRARIES})

add_executable(fact src/factor.cpp)
target_link_libraries(fact ${catkin_LIBRARIES})
