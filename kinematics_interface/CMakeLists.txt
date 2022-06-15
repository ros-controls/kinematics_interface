cmake_minimum_required(VERSION 3.5)
project(kinematics_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


# We require a differential inverse kinematics plugin for converting Cartesian admittance to joint admittance
# Base class for the IK plugin
add_library(kinematics_interface SHARED
        src/kinematics_interface_base.cpp
        )

target_include_directories(
        kinematics_interface
  PUBLIC
  include
)


ament_target_dependencies(
        kinematics_interface
        rclcpp_lifecycle
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(kinematics_interface PRIVATE "KINEMATICS_INTERFACE_BUILDING_DLL")

install(DIRECTORY include/
        DESTINATION include
        )
install(TARGETS kinematics_interface
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )


ament_export_dependencies(
        rclcpp_lifecycle
        sensor_msgs
        trajectory_msgs
)

ament_export_include_directories(
        include
)
ament_export_libraries(
        kinematics_interface
)
ament_package()