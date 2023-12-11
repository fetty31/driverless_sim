include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS})

link_directories(
        ${catkin_LIBRARY_DIRS}
)

add_library(gz_formula_car src/formula_car_plugin.cpp
                        src/vehicle.cpp
                        src/wheel.cpp 
                        src/axle.cpp
                        src/utils/params.cpp
)

target_link_libraries(gz_formula_car ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS gz_formula_car
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
