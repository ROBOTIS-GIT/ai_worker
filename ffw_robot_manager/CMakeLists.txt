cmake_minimum_required(VERSION 3.16)
project(ffw_robot_manager)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  std_srvs
  dynamixel_hardware_interface
  dynamixel_interfaces
  sensor_msgs
  generate_parameter_library
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(ffw_robot_manager_parameters
  src/ffw_robot_manager_parameters.yaml
)

add_library(ffw_robot_manager SHARED
  src/ffw_robot_manager.cpp
  src/battery_model.cpp
  src/ubetter_battery_model.cpp
)
target_compile_features(ffw_robot_manager PUBLIC cxx_std_17)
target_include_directories(ffw_robot_manager PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/ffw_robot_manager>
)

target_link_libraries(ffw_robot_manager PUBLIC
  controller_interface::controller_interface
  ffw_robot_manager_parameters
  pluginlib::pluginlib
  rclcpp::rclcpp
  ${dynamixel_hardware_interface_TARGETS}
  ${std_srvs_TARGETS}
  ${dynamixel_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface ffw_robot_manager_plugin.xml)

install(
  TARGETS ffw_robot_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/ffw_robot_manager
)
install(
  TARGETS
    ffw_robot_manager
    ffw_robot_manager_parameters
  EXPORT export_ffw_robot_manager
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_ffw_robot_manager HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
