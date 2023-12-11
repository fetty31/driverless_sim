include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS})

link_directories(
        ${catkin_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} src/simple_world_plugin.cpp)
add_library(model_push src/simple_physics_plugin.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(model_push ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# install(TARGETS gz_formula_car
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
