cmake_minimum_required(VERSION 3.16)
project(kinematics_interface LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED NO_MODULE)

# Create interface library for kinematics base class
add_library(kinematics_interface SHARED src/kinematics_interface.cpp)
target_compile_features(kinematics_interface PUBLIC cxx_std_17)
target_include_directories(kinematics_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/kinematics_interface>
)
target_link_libraries(kinematics_interface PUBLIC
  Eigen3::Eigen rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle
)

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
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Eigen3)
ament_package()
