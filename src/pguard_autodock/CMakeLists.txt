cmake_minimum_required(VERSION 3.8)
project(pguard_autodock)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(autodock_node src/autodock_node.cpp)
ament_target_dependencies(autodock_node
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  autodock_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/camera_frame_fix.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  models
  scripts
  DESTINATION share/${PROJECT_NAME}
)


# manuelle (export dans .bashrc). Meme mecanisme que turtlebot3_gazebo.
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hook/pguard_autodock_gazebo_paths.sh.in"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
