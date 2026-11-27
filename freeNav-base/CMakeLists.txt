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

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(G2O REQUIRED)
find_package(Glog REQUIRED)
find_package(SUITESPARSE REQUIRED)
find_package(Octomap REQUIRED)
find_package(Pangolin REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options system filesystem thread graph)
#find_package(OMPL REQUIRED)
find_package(LibXml2)
find_package(argparse)


if( NOT LIBXML2_FOUND )
    message( FATAL_ERROR "libxml2 not found\n" )
    return()
endif()

find_package(Python3 COMPONENTS Interpreter Development)

message ( STATUS "Python3_FOUND = ${Python3_FOUND} " )
message ( STATUS "Python3_INCLUDE_DIRS = ${Python3_INCLUDE_DIRS} " )
message ( STATUS "Python3_LIBRARIES = ${Python3_LIBRARIES} " )

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})
include_directories(${OMPL_INCLUDE_DIRS})
include_directories(${CGAL_INCLUDE_DIRS})
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${Python3_INCLUDE_DIRS})
include_directories(${argparse_INCLUDE_DIRS})

if(POLICY CMP0072)
    set(OpenGL_GL_PREFERENCE "GLVND")
endif()
#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)
#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
find_package(Octomap REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

option(ENABLE_LOG "whether enable log" ON)
IF(ENABLE_LOG)
    message(STATUS "Enable log print for debug" )
    add_definitions( -DENABLE_LOG)
ENDIF(ENABLE_LOG)

include_directories(
        #SYSTEM
        ${CSPARSE_INCLUDE_DIR}
        ${SUITESPARSE_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${OCTOMAP_DIRS}
)

include(GoogleTest)


include_directories(${EIGEN3_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
        ${GLOG_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}
        )

find_package(Boost REQUIRED COMPONENTS program_options system filesystem thread graph)

include(GoogleTest)

include_directories(
                    ${CMAKE_SOURCE_DIR}
                    ${Boost_INCLUDE_DIRS}
)

INSTALL(DIRECTORY basic_elements DESTINATION include/freeNav-base)
INSTALL(DIRECTORY dependencies DESTINATION include/freeNav-base)
INSTALL(DIRECTORY visualization DESTINATION include/freeNav-base)


set(G2O_LIBS
        g2o_cli
        g2o_ext_freeglut_minimal
        g2o_simulator
        g2o_solver_slam2d_linear
        g2o_types_icp
        g2o_types_slam2d
        g2o_core g2o_interface
        g2o_solver_csparse
        g2o_solver_structure_only
        g2o_types_sba
        g2o_types_slam3d
        g2o_csparse_extension
        g2o_opengl_helper
        g2o_solver_dense
        g2o_stuff
        g2o_types_sclam2d
        g2o_parser
        g2o_solver_pcg
        g2o_types_data
        g2o_types_sim3
        cxsparse)


aux_source_directory(visualization/3d_viewer    3d_vis_src)
aux_source_directory(visualization/canvas    canvas_src)
aux_source_directory(visualization/qt_gui    qt_src)


aux_source_directory(basic_elements basic_src)
aux_source_directory(dependencies depen_src)
aux_source_directory(dependencies/2d_grid     2d_grid_src)
aux_source_directory(dependencies/3d_octomap  3d_octomap_src)
aux_source_directory(dependencies/3d_textmap  3d_textmap_src)
aux_source_directory(dependencies/massive_scene_loader  mass_src)


add_library(canvas_ex ${basic_src} ${canvas_src} ${2d_grid_src} ${mass_src})
INSTALL (TARGETS canvas_ex canvas_ex LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
#target_link_libraries(harrt_src_lib ${OpenCV_LIBS} ${CGAL_LIBRARIES})

add_library(freeNav-base ${3d_vis_src} ${canvas_src} ${qt_src}
                    ${basic_src}
        ${depen_src} ${2d_grid_src} ${3d_octomap_src} ${3d_textmap_src} ${mass_src}
        basic_elements/pose_se2.cpp
         )

set(EXTERNAL_LIBS
        ${QT_LIBS}
        ${SUITESPARSE_LIBRARIES}
        ${EIGEN3_LIBS}
        ${OpenCV_LIBS}
        gtest
        glog
        gtest_main
        pthread
        ${G2O_LIBS}
        ${CSPARSE_LIB}
        ${OCTOMAP_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        ${Pangolin_LIBRARIES}
        #${CGAL_LIBRARIES} # cause /usr/local/include/g2o/core/base_fixed_sized_edge.hpp:168:33: error: ‘(1.0e+0 / 2.0000000000000001e-9)’ is not a constant expression
        ${yaml-cpp}
        ${Boost_LIBRARIES}
        #${LIBXML2}
        #${DSL_LIBRARY}
        LibXml2::LibXml2
        ${Python3_LIBRARIES}
        /usr/local/lib/libkahypar.so
        ${argparse_lib}
        )

target_link_libraries(freeNav-base ${EXTERNAL_LIBS})





