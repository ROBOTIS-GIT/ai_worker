cmake_minimum_required(VERSION 3.16)
project(ffw_preset_controller)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetHandPresets.srv"
  "srv/SelectHandPreset.srv"
)

install(
  PROGRAMS scripts/hand_preset_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hand_preset_controller
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
