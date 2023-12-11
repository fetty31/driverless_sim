cmake_minimum_required(VERSION 3.0.2)
project(dv_common)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(FILES 
  CarState.msg 
  CarDynState.msg
  CarCommands.msg
  Float32Stamped.msg
  LapInfo.msg 
  LapInfoArray.msg 
  Cone.msg
  ConeArray.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)