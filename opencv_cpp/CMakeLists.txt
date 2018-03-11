cmake_minimum_required(VERSION 2.8)

project(checkerboard)

find_package(OpenCV REQUIRED)

add_executable(checkerboard checkerboard.cpp)

target_link_libraries(checkerboard ${OpenCV_LIBS})
