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
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(hand_presets test/test_hand_presets.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
