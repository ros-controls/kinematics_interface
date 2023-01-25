cmake_minimum_required(VERSION 3.16)
project(kinematics_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Create interface library for kinematics base class
add_library(kinematics_interface INTERFACE)
target_compile_features(kinematics_interface INTERFACE cxx_std_17)
target_include_directories(kinematics_interface INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/kinematics_interface>
)
target_link_libraries(kinematics_interface INTERFACE
  Eigen3::Eigen
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(kinematics_interface INTERFACE "KINEMATICS_INTERFACE_BUILDING_DLL")

install(
  DIRECTORY include/
  DESTINATION include/kinematics_interface
)
install(
  TARGETS kinematics_interface
  EXPORT export_kinematics_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_kinematics_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  rclcpp
  rclcpp_lifecycle
)
ament_package()
