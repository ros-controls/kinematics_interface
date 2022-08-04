cmake_minimum_required(VERSION 3.8)
project(kinematics_interface_kdl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kinematics_interface REQUIRED)

add_library(kinematics_interface_kdl SHARED
        src/kinematics_interface_kdl.cpp
)

ament_target_dependencies(kinematics_interface_kdl
        kdl_parser
        pluginlib
        Eigen3
        kinematics_interface
)

target_include_directories(
        kinematics_interface_kdl
        PRIVATE
        include
)

pluginlib_export_plugin_description_file(kinematics_interface_kdl kinematics_interface_kdl.xml)

install(
        TARGETS kinematics_interface_kdl
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

  ament_add_gmock(
          test_kinematics_interface_kdl
          test/test_kinematics_interface_kdl.cpp
  )
  target_include_directories(test_kinematics_interface_kdl PRIVATE include)
  ament_target_dependencies(test_kinematics_interface_kdl
          kinematics_interface
          ros2_control_test_assets
          pluginlib
          ros2_control_test_assets
          )

endif()

ament_package()
