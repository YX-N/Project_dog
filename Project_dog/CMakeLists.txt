cmake_minimum_required(VERSION 3.10)
project(MasterStack)

add_subdirectory(thirdparty)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

include_directories(
        thirdparty/soem/soem
        thirdparty/soem/osal
        thirdparty/soem/osal/linux
        thirdparty/soem/oshw/linux
)
include_directories(${CMAKE_SOURCE_DIR})

set(CMAKE_CXX_STANDARD 17)

set(SOURCES motor_control.c
    math_ops.c
    transmit.cpp
    motor_bus.cpp
)
LINK_LIBRARIES(m)
add_library(motor ${SOURCES})
target_include_directories(motor PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(motor soem Threads::Threads)

add_executable(stand_position stand_position.cpp)
target_link_libraries(stand_position motor)

add_executable(jump_position jump_position.cpp)
target_link_libraries(jump_position motor)

add_executable(jump_mit jump_mit.cpp)
target_link_libraries(jump_mit motor)

add_executable(jump_test jump_test.cpp)
target_link_libraries(jump_test motor)

add_executable(read_angles read_angles.cpp)
target_link_libraries(read_angles motor)
