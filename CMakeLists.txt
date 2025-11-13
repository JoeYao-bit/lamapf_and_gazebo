cmake_minimum_required(VERSION 3.10)
project(lamapf_and_gazebo)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message(STATUS "AMENT_PREFIX_PATH: $ENV{AMENT_PREFIX_PATH}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lamapf_and_gazebo_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)


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

set(dependencies rclcpp ros_gz_interfaces tf2 std_msgs geometry_msgs lamapf_and_gazebo_msgs sensor_msgs)        

add_executable(spawn_entity src/test_spawn_entity.cpp)
ament_target_dependencies(spawn_entity ${dependencies})
install(TARGETS spawn_entity DESTINATION lib/${PROJECT_NAME})
target_link_libraries(spawn_entity ${EXTERNAL_LIBS} )


add_executable(test_lamapf_planner_node src/test_lamapf_planner_node.cpp)
target_link_libraries(test_lamapf_planner_node ${EXTERNAL_LIBS} )
ament_target_dependencies(test_lamapf_planner_node ${dependencies})
install(TARGETS test_lamapf_planner_node DESTINATION lib/${PROJECT_NAME})

add_executable(generate_instance_node src/generate_instance.cpp)
target_link_libraries(generate_instance_node ${EXTERNAL_LIBS} )
ament_target_dependencies(generate_instance_node ${dependencies})
install(TARGETS generate_instance_node DESTINATION lib/${PROJECT_NAME})

add_executable(test_central_controller src/test_central_controller.cpp)
target_link_libraries(test_central_controller ${EXTERNAL_LIBS} )
ament_target_dependencies(test_central_controller ${dependencies})
install(TARGETS test_central_controller DESTINATION lib/${PROJECT_NAME})

add_executable(test_mpc_controller src/test_mpc_controller.cpp)
target_link_libraries(test_mpc_controller ${EXTERNAL_LIBS} )
ament_target_dependencies(test_mpc_controller ${dependencies})
install(TARGETS test_mpc_controller DESTINATION lib/${PROJECT_NAME})

add_executable(test_yaml_converter src/test_yaml_converter.cpp)
target_link_libraries(test_yaml_converter ${EXTERNAL_LIBS} yaml-cpp )
ament_target_dependencies(test_yaml_converter ${dependencies})
install(TARGETS test_yaml_converter DESTINATION lib/${PROJECT_NAME})

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
