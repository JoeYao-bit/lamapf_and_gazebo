cmake_minimum_required(VERSION 3.10)
project(lamapf_and_gazebo)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


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
# dwa dependencies
#find_package(pluginlib REQUIRED)
#find_package(nav2_core REQUIRED)
#find_package(nav2_costmap_2d REQUIRED)
#find_package(dwb_core REQUIRED)
#find_package(nav_msgs REQUIRED)
#find_package(dwb_critics REQUIRED)
#find_package(dwb_plugins REQUIRED)
#find_package(nav2_controller REQUIRED)
#find_package(nav2_util REQUIRED)
#find_package(nav2_msgs REQUIRED)

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
 
add_executable(spawn_entity src/test_spawn_entity.cpp)
ament_target_dependencies(spawn_entity ${dependencies})
install(TARGETS spawn_entity DESTINATION lib/${PROJECT_NAME})
target_link_libraries(spawn_entity ${EXTERNAL_LIBS} yaml-cpp)


add_executable(test_lamapf_planner_node src/test_lamapf_planner_node.cpp)
target_link_libraries(test_lamapf_planner_node ${EXTERNAL_LIBS} yaml-cpp)
ament_target_dependencies(test_lamapf_planner_node ${dependencies})
install(TARGETS test_lamapf_planner_node DESTINATION lib/${PROJECT_NAME})

add_executable(generate_instance_node src/generate_instance.cpp)
target_link_libraries(generate_instance_node ${EXTERNAL_LIBS} yaml-cpp)
ament_target_dependencies(generate_instance_node ${dependencies})
install(TARGETS generate_instance_node DESTINATION lib/${PROJECT_NAME})

add_executable(test_central_controller src/test_central_controller.cpp)
target_link_libraries(test_central_controller ${EXTERNAL_LIBS} yaml-cpp)
ament_target_dependencies(test_central_controller ${dependencies})
install(TARGETS test_central_controller DESTINATION lib/${PROJECT_NAME})

add_executable(test_mpc_controller src/test_mpc_controller.cpp)
target_link_libraries(test_mpc_controller ${EXTERNAL_LIBS} yaml-cpp)
ament_target_dependencies(test_mpc_controller ${dependencies})
install(TARGETS test_mpc_controller DESTINATION lib/${PROJECT_NAME})

add_executable(test_yaml_converter src/test_yaml_converter.cpp)
target_link_libraries(test_yaml_converter ${EXTERNAL_LIBS} yaml-cpp )
ament_target_dependencies(test_yaml_converter ${dependencies})
install(TARGETS test_yaml_converter DESTINATION lib/${PROJECT_NAME})

add_executable(test_local_controller src/test_local_controller.cpp)
target_link_libraries(test_local_controller ${EXTERNAL_LIBS} yaml-cpp )
ament_target_dependencies(test_local_controller ${dependencies})
install(TARGETS test_local_controller DESTINATION lib/${PROJECT_NAME})

add_executable(test_center_controller_in_read_world src/test_center_controller_in_real_world.cpp)
target_link_libraries(test_center_controller_in_read_world ${EXTERNAL_LIBS} yaml-cpp)
ament_target_dependencies(test_center_controller_in_read_world ${dependencies})
install(TARGETS test_center_controller_in_read_world DESTINATION lib/${PROJECT_NAME})

add_executable(test_dwa_local_planner src/test_dwa_local_planner.cpp)
target_link_libraries(test_dwa_local_planner ${EXTERNAL_LIBS} yaml-cpp)
ament_target_dependencies(test_dwa_local_planner ${dependencies})
install(TARGETS test_dwa_local_planner DESTINATION lib/${PROJECT_NAME})

add_executable(test_center_controller_multiple_agent src/test_center_controller_multiple_agent.cpp)
target_link_libraries(test_center_controller_multiple_agent ${EXTERNAL_LIBS} yaml-cpp)
ament_target_dependencies(test_center_controller_multiple_agent ${dependencies})
install(TARGETS test_center_controller_multiple_agent DESTINATION lib/${PROJECT_NAME})

add_executable(test_raise_local_controller src/test_raise_local_controller.cpp)
target_link_libraries(test_raise_local_controller ${EXTERNAL_LIBS} yaml-cpp)
ament_target_dependencies(test_raise_local_controller ${dependencies})
install(TARGETS test_raise_local_controller DESTINATION lib/${PROJECT_NAME})

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
