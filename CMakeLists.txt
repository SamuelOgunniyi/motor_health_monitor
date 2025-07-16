cmake_minimum_required(VERSION 3.10)
project(motor_health_monitor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

set(SOURCES
    src/main.cpp
    src/CmdOdomSync.cpp
)

add_executable(motor_health_monitor ${SOURCES})

find_package(Threads REQUIRED)
target_link_libraries(motor_health_monitor Threads::Threads)

