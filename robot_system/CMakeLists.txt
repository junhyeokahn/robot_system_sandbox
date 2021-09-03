file(GLOB sources "*.cpp" "*.c")
file(GLOB robot_system_headers "*.hpp")

add_library(my_robot_system ${sources}
                             ${robot_system_headers})

target_link_libraries(my_robot_system ${DART_LIBRARIES}
                                      ${PINOCCHIO_LIBRARIES})
