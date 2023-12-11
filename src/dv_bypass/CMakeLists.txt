cmake_minimum_required(VERSION 3.0.2)
project(dv_bypass)

## Compile as C++11
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dv_common
  roscpp
  std_msgs
  message_filters
  tf 
  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS dv_common roscpp std_msgs message_filters tf geometry_msgs 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp 
)

set_target_properties(${PROJECT_NAME} PROPERTIES ENABLE_EXPORTS 1)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})