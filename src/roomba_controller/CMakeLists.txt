cmake_minimum_required(VERSION 3.16)
project(roomba_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(roomba_controller_parameters
  src/roomba_controller_parameter.yaml
)

add_library(roomba_controller SHARED
  src/roomba_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)
target_compile_features(roomba_controller PUBLIC cxx_std_17)
target_include_directories(roomba_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/roomba_controller>
)
target_link_libraries(roomba_controller PUBLIC roomba_controller_parameters)
ament_target_dependencies(roomba_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(roomba_controller PRIVATE "ROOMBA_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface roomba_plugin.xml)


install(
  DIRECTORY include/
  DESTINATION include/roomba_controller
)
install(TARGETS roomba_controller roomba_controller_parameters
  EXPORT export_roomba_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_roomba_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
