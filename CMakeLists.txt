cmake_minimum_required(VERSION 3.5)

project(OpenCV-AR LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package( OpenCV REQUIRED )

add_executable(OpenCV-AR main.cpp)
target_link_libraries( OpenCV-AR ${OpenCV_LIBS} )
