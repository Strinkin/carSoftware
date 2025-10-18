cmake_minimum_required(VERSION 3.10)
project(car_software3)

set(CMAKE_CXX_STANDARD 17)
file(GLOB SRCS src/*.cpp)

add_executable(main ${SRCS})
target_include_directories(main PRIVATE include)