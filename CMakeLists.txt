cmake_minimum_required(VERSION 3.8)
project(laser_scan_unifier)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra 
  -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow 
  -Wno-float-conversion -Wno-sign-compare)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  generate_parameter_library
  laser_geometry
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  tf2
)
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(laser_scan_unifier_parameters
  src/laser_scan_unifier_parameters.yaml
)

# Add executable
add_executable(laser_scan_unifier src/laser_scan_unifier.cpp)
target_include_directories(laser_scan_unifier PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/laser_scan_unifier>
)
target_link_libraries(laser_scan_unifier PUBLIC
  laser_scan_unifier_parameters
)
ament_target_dependencies(laser_scan_unifier PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  DIRECTORY include/
  DESTINATION include/laser_scan_unifier
)
install(
  TARGETS laser_scan_unifier_parameters
  EXPORT export_laser_scan_unifier
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(
  TARGETS laser_scan_unifier
  RUNTIME DESTINATION lib/laser_scan_unifier
)

ament_export_targets(export_laser_scan_unifier HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
ament_generate_version_header(${PROJECT_NAME})


