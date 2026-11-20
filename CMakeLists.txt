cmake_minimum_required(VERSION 3.10)
project(lamapf_and_gazebo)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(
  -w   # 👈 关闭所有 warning
)

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
world
DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
map
DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
config
DESTINATION share/${PROJECT_NAME}/
)

message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message(STATUS "AMENT_PREFIX_PATH: $ENV{AMENT_PREFIX_PATH}")
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config

)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lamapf_and_gazebo_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators)


ament_export_dependencies(rosidl_default_runtime)



set(EXTERNAL_LIBS
        libcanvas_ex.a
        liblamapf_alg_ex.a
        pthread
        ${OpenCV_LIBS}
        libtinyxml.so
        )


include_directories(include 
                    ${EIGEN3_INCLUDE_DIRS}   # 把 Eigen 头文件路径加进来
                    )

set(dependencies rclcpp ros_gz_interfaces tf2 tf2_ros tf2_geometry_msgs std_msgs geometry_msgs lamapf_and_gazebo_msgs sensor_msgs rclcpp_lifecycle ament_index_cpp
#pluginlib nav2_core nav2_costmap_2d dwb_core dwb_critics dwb_plugins nav_msgs nav2_controller nav2_msgs nav2_util
  )        
 

add_executable(test_lamapf_planner_node src/test_lamapf_planner_node.cpp)
target_link_libraries(test_lamapf_planner_node ${EXTERNAL_LIBS} yaml-cpp)
ament_target_dependencies(test_lamapf_planner_node ${dependencies})
install(TARGETS test_lamapf_planner_node DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
