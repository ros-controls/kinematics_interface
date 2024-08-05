cmake_minimum_required(VERSION 3.16)
project(kinematics_interface_pinocchio LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  kinematics_interface
  pluginlib
  eigen3_cmake_module
  pinocchio
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(
  kinematics_interface_pinocchio
  SHARED
  src/kinematics_interface_pinocchio.cpp
)
target_include_directories(kinematics_interface_pinocchio PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/kinematics_interface_pinocchio>
)
target_compile_features(kinematics_interface_pinocchio PUBLIC cxx_std_17)
ament_target_dependencies(kinematics_interface_pinocchio PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(kinematics_interface_pinocchio PUBLIC
  Eigen3::Eigen pinocchio::pinocchio
)

pluginlib_export_plugin_description_file(kinematics_interface kinematics_interface_pinocchio.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_kinematics_interface_pinocchio
    test/test_kinematics_interface_pinocchio.cpp
  )
  target_link_libraries(test_kinematics_interface_pinocchio kinematics_interface_pinocchio)

  # TODO: Use target_link_libraries once ros2_control_test_assets'
  #       CMake include export is fixed
  ament_target_dependencies(test_kinematics_interface_pinocchio ros2_control_test_assets)
endif()

install(
  DIRECTORY include/
  DESTINATION include/kinematics_interface_pinocchio
)
install(
  TARGETS kinematics_interface_pinocchio
  EXPORT export_kinematics_interface_pinocchio
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_kinematics_interface_pinocchio HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
