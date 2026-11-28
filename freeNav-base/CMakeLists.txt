cmake_minimum_required(VERSION 3.5)

project(freeNav-base)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
enable_testing()
# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(STATUS "Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -std=c++14")
    add_definitions(-DCOMPILEDWITHC14)
    message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

#set(OpenCV_INCLUDE_DIRS "/usr/include/opencv4")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options system filesystem thread graph)
#find_package(OMPL REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})


include_directories(
        ${CMAKE_SOURCE_DIR}
        )

find_package(Boost REQUIRED COMPONENTS program_options system filesystem thread graph)


include_directories(
                    ${CMAKE_SOURCE_DIR}
                    ${Boost_INCLUDE_DIRS}
)

INSTALL(DIRECTORY basic_elements DESTINATION include/freeNav-base)
INSTALL(DIRECTORY dependencies DESTINATION include/freeNav-base)
INSTALL(DIRECTORY visualization DESTINATION include/freeNav-base)


aux_source_directory(visualization/canvas    canvas_src)


aux_source_directory(basic_elements basic_src)
aux_source_directory(dependencies depen_src)
aux_source_directory(dependencies/2d_grid     2d_grid_src)
aux_source_directory(dependencies/3d_octomap  3d_octomap_src)
aux_source_directory(dependencies/3d_textmap  3d_textmap_src)
aux_source_directory(dependencies/massive_scene_loader  mass_src)


add_library(canvas_ex ${basic_src} ${canvas_src} ${2d_grid_src} ${mass_src})
INSTALL (TARGETS canvas_ex canvas_ex LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
#target_link_libraries(harrt_src_lib ${OpenCV_LIBS} ${CGAL_LIBRARIES})

add_library(freeNav-base ${canvas_src}
                    ${basic_src}
        ${depen_src} ${2d_grid_src} ${3d_octomap_src} ${3d_textmap_src} ${mass_src}
         )

set(EXTERNAL_LIBS
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        )

target_link_libraries(freeNav-base ${EXTERNAL_LIBS})





