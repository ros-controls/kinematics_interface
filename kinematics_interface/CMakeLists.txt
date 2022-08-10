cmake_minimum_required(VERSION 3.5)
project(kinematics_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_lifecycle
    Eigen3
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Create interface library for kinematics base class
add_library(kinematics_interface INTERFACE)
target_include_directories(kinematics_interface INTERFACE include)
ament_target_dependencies(kinematics_interface INTERFACE ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(kinematics_interface INTERFACE "KINEMATICS_INTERFACE_BUILDING_DLL")

install(DIRECTORY include/
    DESTINATION include
)

ament_export_dependencies(
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_include_directories(
    include
)

ament_package()
