cmake_minimum_required(VERSION 3.16)
project(kinematics_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  Eigen3
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Create interface library for kinematics base class
add_library(kinematics_interface INTERFACE)
target_compile_features(kinematics_interface INTERFACE cxx_std_17)
target_include_directories(kinematics_interface INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/kinematics_interface>
)
ament_target_dependencies(kinematics_interface INTERFACE
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
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
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
