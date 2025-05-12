################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.16)
project(ffw_joystick_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
                      -Werror=missing-braces)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

################################################################################
# Build
################################################################################
generate_parameter_library(joystick_controller_parameters
  src/joystick_controller_parameters.yaml
)

add_library(joystick_controller SHARED
  src/joystick_controller.cpp
)
target_compile_features(joystick_controller PUBLIC cxx_std_17)
target_include_directories(joystick_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/joystick_controller>
)
target_link_libraries(joystick_controller PUBLIC
  ${std_msgs_TARGETS} controller_interface::controller_interface hardware_interface::mock_components joystick_controller_parameters pluginlib::pluginlib rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(joystick_controller PRIVATE "joystick_controller_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface joystick_plugin.xml)

################################################################################
# Install
################################################################################
install(
  DIRECTORY include/
  DESTINATION include/joystick_controller
)
install(TARGETS
    joystick_controller
    joystick_controller_parameters
  EXPORT export_joystick_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

################################################################################
# Export information
################################################################################
ament_export_targets(export_joystick_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

################################################################################
# Macro for ament package
################################################################################
ament_package()
