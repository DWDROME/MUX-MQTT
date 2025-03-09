cmake_minimum_required(VERSION 3.0.2)
project(learning_tf)

## 先查找 catkin 组件
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  turtlesim
)

## 让 catkin_package 知道我们用到了哪些依赖
catkin_package(
  CATKIN_DEPENDS roscpp rospy tf turtlesim
)

## 头文件路径
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 添加可执行文件
add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)

## 连接 ROS 库
target_link_libraries(turtle_tf_broadcaster ${catkin_LIBRARIES})
