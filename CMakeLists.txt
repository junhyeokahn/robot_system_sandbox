cmake_minimum_required(VERSION 3.5.1)

project(robot_system)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS on)

find_package(DART 6.10 REQUIRED COMPONENTS utils-urdf gui-osg CONFIG)
include_directories(${DART_INCLUDE_DIRS})

find_package(pinocchio)
include_directories(${PINOCCHIO_INCLUDE_DIRS})

include_directories("${PROJECT_SOURCE_DIR}")

add_subdirectory(robot_system)

add_executable(main main.cpp)
target_link_libraries(main my_robot_system)

add_executable(main2 main2.cpp)
target_link_libraries(main2 my_robot_system)

add_executable(main3 main3.cpp)
target_link_libraries(main3 my_robot_system)

add_executable(main4 main4.cpp)
target_link_libraries(main4 my_robot_system)
