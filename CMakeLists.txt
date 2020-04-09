cmake_minimum_required(VERSION 3.5)

project(opencv-ar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package( OpenCV REQUIRED )

add_executable(opencv-ar main.cpp)
target_link_libraries( opencv-ar ${OpenCV_LIBS} )
