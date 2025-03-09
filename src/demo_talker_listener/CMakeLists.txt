cmake_minimum_required(VERSION 3.0.2)
project(demo_talker_listener)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(hellow src/hellow.cpp)
target_link_libraries(hellow ${catkin_LIBRARIES})

