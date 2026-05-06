cmake_minimum_required(VERSION 3.8)
project(ffw_swerve_drive_controller)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  generate_parameter_library
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  sensor_msgs
  visualization_msgs
  rcpputils
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
add_compile_definitions(RCPPUTILS_VERSION_MAJOR=${rcpputils_VERSION_MAJOR})
add_compile_definitions(RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR})

generate_parameter_library(swerve_drive_controller_parameter
  config/swerve_drive_controller_parameter.yaml
)


add_library(ffw_swerve_drive_controller
  SHARED
  src/swerve_drive_controller.cpp
  src/odometry.cpp
  src/marker_visualize.cpp
  src/speed_limiter.cpp
)
target_compile_features(ffw_swerve_drive_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(ffw_swerve_drive_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${controller_interface_INCLUDE_DIRS}
  ${hardware_interface_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${realtime_tools_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${tf2_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${rcpputils_INCLUDE_DIRS}
)

target_link_libraries(ffw_swerve_drive_controller PUBLIC
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  ${generate_parameter_library_TARGETS}
  ${nav_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  ${tf2_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  rcpputils::rcpputils
  swerve_drive_controller_parameter
)

# Export the plugin definition file
pluginlib_export_plugin_description_file(controller_interface ffw_swerve_drive_controller.xml) # Use your XML filename

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ffw_swerve_drive_controller PRIVATE "SWERVE_DRIVE_CONTROLLER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ffw_swerve_drive_controller swerve_drive_controller_parameter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install the plugin XML file
install(FILES ffw_swerve_drive_controller.xml # Use your XML filename
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ffw_swerve_drive_controller
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


ament_package()
