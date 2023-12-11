cmake_minimum_required(VERSION 3.0.2)
project(dv_tracker)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  dv_common
  gazebo_ros
  roscpp
  std_msgs
  tf2
  geometry_msgs
  message_generation
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

add_service_files(
  FILES 
    Init.srv 
)

generate_messages(
  DEPENDENCIES 
    std_msgs
)

link_directories(
        ${catkin_LIBRARY_DIRS}
        ${GAZEBO_LIBRARY_DIRS}
)

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS
    roscpp
    gazebo_ros
    std_msgs
    tf2
    geometry_msgs
    visualization_msgs
    message_runtime
    dv_common
)
         

add_library(gz_tracker src/tracker_plugin.cpp
                        src/accumulator.cpp
                        src/tracker.cpp
)

target_link_libraries(gz_tracker ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS gz_tracker
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
