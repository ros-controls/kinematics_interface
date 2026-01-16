cmake_minimum_required(VERSION 3.16)
project(kinematics_interface_ikfast LANGUAGES CXX)

# Default ros2_control compiler settings
find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# Dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  kinematics_interface
  pluginlib
  tf2_eigen
  rclcpp
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED NO_MODULE)


add_library(
  kinematics_interface_ikfast
  SHARED
  src/kinematics_interface_ikfast.cpp
)

target_include_directories(kinematics_interface_ikfast PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(kinematics_interface_ikfast PUBLIC cxx_std_17)

target_link_libraries(kinematics_interface_ikfast PUBLIC
  Eigen3::Eigen
  kinematics_interface::kinematics_interface
  pluginlib::pluginlib
  tf2_eigen::tf2_eigen
  rclcpp::rclcpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(
    test_kinematics_interface_ikfast
    test/test_kinematics_interface_ikfast.cpp
  )

  target_include_directories(test_kinematics_interface_ikfast PRIVATE
    include
    src
  )

  target_link_libraries(test_kinematics_interface_ikfast
    kinematics_interface_ikfast
    kinematics_interface::kinematics_interface_tests
  )
endif()


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS kinematics_interface_ikfast
  EXPORT export_kinematics_interface_ikfast
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_kinematics_interface_ikfast HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Eigen3)

ament_package()