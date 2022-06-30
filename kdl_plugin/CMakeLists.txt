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

add_library(kdl_plugin SHARED
        src/kdl_plugin.cpp
        )

ament_target_dependencies(kdl_plugin
        kdl_parser
        pluginlib
        Eigen3
        kinematics_interface
        )

target_include_directories(
        kdl_plugin
        PRIVATE
        include
)

pluginlib_export_plugin_description_file(kdl_plugin kdl_plugin.xml)

install(
        TARGETS kdl_plugin
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
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
          test_kdl_plugin
          test/test_kdl_plugin.cpp
  )
  target_include_directories(test_kdl_plugin PRIVATE include)
  ament_target_dependencies(test_kdl_plugin
          kinematics_interface
          ros2_control_test_assets
          pluginlib
          ros2_control_test_assets
          )

endif()

ament_package()
