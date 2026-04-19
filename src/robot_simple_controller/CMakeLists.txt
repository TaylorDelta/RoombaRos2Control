cmake_minimum_required(VERSION 3.8)
project(robot_simple_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  nav_msgs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add robot_simple_controller library related compile commands
generate_parameter_library(robot_simple_controller_parameters
  src/robot_simple_controller.yaml
  include/robot_simple_controller/validate_robot_simple_controller_parameters.hpp
)
add_library(
  robot_simple_controller
  SHARED
  src/robot_simple_controller.cpp
)
target_include_directories(robot_simple_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(robot_simple_controller robot_simple_controller_parameters)
ament_target_dependencies(robot_simple_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(robot_simple_controller PRIVATE "ROBOT_SIMPLE_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface robot_simple_controller.xml)

install(
  TARGETS
  robot_simple_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  robot_simple_controller
)

ament_package()
