cmake_minimum_required(VERSION 3.16)
project(ffw_calibration LANGUAGES C CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(msg_files
  "msg/CalibrationStatus.msg"
)

set(srv_files
  "srv/ApplyEffort.srv"
  "srv/CaptureJoint.srv"
  "srv/GetCalibrationConfig.srv"
  "srv/GetHomingOffsets.srv"
  "srv/MoveArmTrajectory.srv"
  "srv/MoveToZeroPose.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(calibration_service
  src/calibration_service.cpp
)
target_compile_features(calibration_service PUBLIC cxx_std_17)
target_include_directories(calibration_service PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(calibration_service
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
)
target_link_libraries(calibration_service
  "${cpp_typesupport_target}"
)

install(
  TARGETS calibration_service
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
