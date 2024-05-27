cmake_minimum_required(VERSION 3.8)
project(laser_scan_unifier)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra 
  -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow 
  -Wno-float-conversion -Wno-sign-compare)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  laser_geometry
  generate_parameter_library
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  tf2
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add scan_unifier_node library related compile commands
generate_parameter_library(scan_unifier_node_parameters
  src/scan_unifier_node.yaml
)
add_executable(scan_unifier_node src/scan_unifier_node.cpp)
target_include_directories(scan_unifier_node PRIVATE
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(scan_unifier_node scan_unifier_node_parameters)
ament_target_dependencies(scan_unifier_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS
  scan_unifier_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_libraries(
  scan_unifier_node
)

ament_package()


