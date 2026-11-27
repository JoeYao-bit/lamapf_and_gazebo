cmake_minimum_required(VERSION 3.17)
project(lamapf_and_center_centroller)

set(CMAKE_CXX_STANDARD 17)

set(OpenCV_INCLUDE_DIRS "/usr/include/opencv4")
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(yaml-cpp REQUIRED)

add_subdirectory(freeNav-base)

set(EXTERNAL_LIBS
        libcanvas_ex.a
        liblamapf_alg_ex.a
        pthread
        yaml-cpp
        freeNav-base
        )

add_executable(lamapf_and_center_controller main.cpp)
target_link_libraries(lamapf_and_center_controller ${EXTERNAL_LIBS})