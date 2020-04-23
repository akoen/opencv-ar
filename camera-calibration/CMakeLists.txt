cmake_minimum_required(VERSION 3.5)

project(camera-calibration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package( OpenCV REQUIRED )

add_executable(camera-calibration main.cpp)
target_link_libraries( camera-calibration ${OpenCV_LIBS} )
