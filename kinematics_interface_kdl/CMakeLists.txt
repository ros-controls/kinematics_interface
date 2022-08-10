cmake_minimum_required(VERSION 3.8)
project(kinematics_interface_kdl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  Eigen3
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

add_library(
  ${PROJECT_NAME}
  SHARED
  src/kinematics_interface_kdl.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(kinematics_interface kinematics_interface_kdl.xml)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)

  ament_add_gmock(
    test_${PROJECT_NAME}
    test/test_kinematics_interface_kdl.cpp
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE include)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
  ament_target_dependencies(test_${PROJECT_NAME}
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
    ros2_control_test_assets
  )
endif()

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
