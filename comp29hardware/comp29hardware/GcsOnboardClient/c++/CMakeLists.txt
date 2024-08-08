cmake_minimum_required(VERSION 3.27)
project(Onboard_C)

set(CMAKE_CXX_STANDARD 17)
include_directories(./include
)
add_executable(Onboard_C main.cpp
        src/UavOnboard.cpp)
target_link_libraries(Onboard_C ws2_32)