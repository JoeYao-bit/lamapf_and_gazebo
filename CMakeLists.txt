cmake_minimum_required(VERSION 3.17)
project(lamapf_and_center_centroller)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)

set(OpenCV_INCLUDE_DIRS "/usr/include/opencv4")
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(freeNav-base)

set(EXTERNAL_LIBS
        libcanvas_ex.a
        )

set(SRC_FILES
        LayeredMAPF/algorithm/constraint_table_CBS/common.cpp
        LayeredMAPF/algorithm/constraint_table_CBS/Conflict.cpp
        LayeredMAPF/algorithm/LA-MAPF/IndependenceDetection/independence_detection.cpp
        LayeredMAPF/algorithm/LA-MAPF/block_shaped_agent.cpp
        LayeredMAPF/algorithm/LA-MAPF/circle_shaped_agent.cpp
        LayeredMAPF/algorithm/LA-MAPF/common.cpp
        LayeredMAPF/algorithm/basic.cpp
        LayeredMAPF/algorithm/instance_decomposition.cpp
        )

add_executable(lamapf_and_center_controller main.cpp ${SRC_FILES})
target_link_libraries(lamapf_and_center_controller ${EXTERNAL_LIBS} ${OpenCV_LIBS} )