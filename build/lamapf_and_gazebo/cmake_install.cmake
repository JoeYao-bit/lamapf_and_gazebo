# Install script for directory: /home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/install/lamapf_and_gazebo")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lamapf_and_gazebo/" TYPE DIRECTORY FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/launch")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/spawn_entity" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/spawn_entity")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/spawn_entity"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo" TYPE EXECUTABLE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/spawn_entity")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/spawn_entity" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/spawn_entity")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/spawn_entity"
         OLD_RPATH "/opt/ros/jazzy/lib:/home/yaozhuo/code/ros2_ws/install/lamapf_and_gazebo_msgs/lib:/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/spawn_entity")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/CMakeFiles/spawn_entity.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_lamapf_planner_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_lamapf_planner_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_lamapf_planner_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo" TYPE EXECUTABLE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/test_lamapf_planner_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_lamapf_planner_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_lamapf_planner_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_lamapf_planner_node"
         OLD_RPATH "/usr/local/lib:/opt/ros/jazzy/lib:/home/yaozhuo/code/ros2_ws/install/lamapf_and_gazebo_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_lamapf_planner_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/CMakeFiles/test_lamapf_planner_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/generate_instance_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/generate_instance_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/generate_instance_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo" TYPE EXECUTABLE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/generate_instance_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/generate_instance_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/generate_instance_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/generate_instance_node"
         OLD_RPATH "/usr/local/lib:/opt/ros/jazzy/lib:/home/yaozhuo/code/ros2_ws/install/lamapf_and_gazebo_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/generate_instance_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/CMakeFiles/generate_instance_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_central_controller" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_central_controller")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_central_controller"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo" TYPE EXECUTABLE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/test_central_controller")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_central_controller" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_central_controller")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_central_controller"
         OLD_RPATH "/usr/local/lib:/opt/ros/jazzy/lib:/home/yaozhuo/code/ros2_ws/install/lamapf_and_gazebo_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_central_controller")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/CMakeFiles/test_central_controller.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_mpc_controller" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_mpc_controller")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_mpc_controller"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo" TYPE EXECUTABLE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/test_mpc_controller")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_mpc_controller" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_mpc_controller")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_mpc_controller"
         OLD_RPATH "/usr/local/lib:/opt/ros/jazzy/lib:/home/yaozhuo/code/ros2_ws/install/lamapf_and_gazebo_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_mpc_controller")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/CMakeFiles/test_mpc_controller.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_yaml_converter" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_yaml_converter")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_yaml_converter"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo" TYPE EXECUTABLE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/test_yaml_converter")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_yaml_converter" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_yaml_converter")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_yaml_converter"
         OLD_RPATH "/usr/local/lib:/opt/ros/jazzy/lib:/home/yaozhuo/code/ros2_ws/install/lamapf_and_gazebo_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_yaml_converter")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/CMakeFiles/test_yaml_converter.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_local_controller" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_local_controller")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_local_controller"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo" TYPE EXECUTABLE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/test_local_controller")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_local_controller" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_local_controller")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_local_controller"
         OLD_RPATH "/usr/local/lib:/opt/ros/jazzy/lib:/home/yaozhuo/code/ros2_ws/install/lamapf_and_gazebo_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lamapf_and_gazebo/test_local_controller")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/CMakeFiles/test_local_controller.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/lamapf_and_gazebo")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/lamapf_and_gazebo")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lamapf_and_gazebo/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lamapf_and_gazebo/environment" TYPE FILE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lamapf_and_gazebo/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lamapf_and_gazebo/environment" TYPE FILE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lamapf_and_gazebo" TYPE FILE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lamapf_and_gazebo" TYPE FILE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lamapf_and_gazebo" TYPE FILE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lamapf_and_gazebo" TYPE FILE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lamapf_and_gazebo" TYPE FILE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/ament_cmake_index/share/ament_index/resource_index/packages/lamapf_and_gazebo")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lamapf_and_gazebo/cmake" TYPE FILE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lamapf_and_gazebo/cmake" TYPE FILE FILES
    "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/ament_cmake_core/lamapf_and_gazeboConfig.cmake"
    "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/ament_cmake_core/lamapf_and_gazeboConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lamapf_and_gazebo" TYPE FILE FILES "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/package.xml")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/build/lamapf_and_gazebo/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
