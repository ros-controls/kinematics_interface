cmake_minimum_required(VERSION 3.16)
project(kinematics_interface_kdl LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  kdl_parser
  kinematics_interface
  pluginlib
  tf2_eigen_kdl
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(
  kinematics_interface_kdl
  SHARED
  src/kinematics_interface_kdl.cpp
)
target_include_directories(kinematics_interface_kdl PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/kinematics_interface_kdl>
)
target_compile_features(kinematics_interface_kdl PUBLIC cxx_std_17)
target_link_libraries(kinematics_interface_kdl PUBLIC
  Eigen3::Eigen kdl_parser::kdl_parser kinematics_interface::kinematics_interface pluginlib::pluginlib tf2_eigen_kdl::tf2_eigen_kdl
)

pluginlib_export_plugin_description_file(kinematics_interface kinematics_interface_kdl.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_kinematics_interface_kdl
    test/test_kinematics_interface_kdl.cpp
  )
  target_link_libraries(test_kinematics_interface_kdl kinematics_interface_kdl ros2_control_test_assets::ros2_control_test_assets)
endif()

install(
  DIRECTORY include/
  DESTINATION include/kinematics_interface_kdl
)
install(
  TARGETS kinematics_interface_kdl
  EXPORT export_kinematics_interface_kdl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_kinematics_interface_kdl HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Eigen3)
ament_package()
