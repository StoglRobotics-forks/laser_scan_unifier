cmake_minimum_required(VERSION 3.8)
project(laser_scan_unifier)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  laser_geometry
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add library for the header file
add_library(
  laser_scan_unifier 
  SHARED
  src/scan_unifier_node.cpp
)
# Add executable
add_executable(scan_unifier_node src/scan_unifier_node.cpp)

target_include_directories(
  laser_scan_unifier
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
ament_target_dependencies(
  laser_scan_unifier PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# Link the library to the executable
target_link_libraries(scan_unifier_node laser_scan_unifier)


install(
  DIRECTORY include/
  DESTINATION include
)

# Install
install(TARGETS
  scan_unifier_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


