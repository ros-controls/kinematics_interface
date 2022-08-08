cmake_minimum_required(VERSION 3.5)
project(kinematics_interface)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Create interface library for kinematics base class
add_library(kinematics_interface
    INTERFACE
)

target_include_directories(
    kinematics_interface
    INTERFACE
    include
)

ament_target_dependencies(
    kinematics_interface
    INTERFACE
    rclcpp_lifecycle
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(kinematics_interface INTERFACE "KINEMATICS_INTERFACE_BUILDING_DLL")

install(DIRECTORY include/
    DESTINATION include
)

ament_export_dependencies(
    rclcpp_lifecycle
)

ament_export_include_directories(
    include
)

ament_package()
