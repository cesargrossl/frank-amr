cmake_minimum_required(VERSION 3.10)
project(LidarSocket)

add_definitions(-std=c++11)

include_directories(sdk/sdk/include)
link_directories(sdk/sdk/lib)

add_executable(lidar_server main.cpp)
target_link_libraries(lidar_server sl_lidar_sdk)
