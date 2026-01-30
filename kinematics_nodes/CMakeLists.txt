cmake_minimum_required(VERSION 3.8)
project(kinematics_nodes)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(kinematics_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)

# Executable: ik_plugin_service_node
add_executable(ik_plugin_service_node src/ik_plugin_service.cpp)

target_include_directories(ik_plugin_service_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(ik_plugin_service_node
  rclcpp
  pluginlib
  kinematics_interface
  moveit_msgs
  geometry_msgs
  sensor_msgs
  tf2_eigen
  urdf
)

# Install executable
install(TARGETS ik_plugin_service_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/test_ik_poses.py
  test/test_kinematics_service.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
