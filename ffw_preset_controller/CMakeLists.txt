cmake_minimum_required(VERSION 3.16)
project(ffw_preset_controller)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/hand_preset_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hand_preset_controller
)

# Share the preset schema/validation with the pose recording executable.
install(FILES scripts/hand_preset_controller.py DESTINATION lib/${PROJECT_NAME})
install(
  PROGRAMS scripts/record_hand_preset.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME record_hand_preset
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(hand_presets test/test_hand_presets.py)
  ament_add_pytest_test(record_hand_presets test/test_record_hand_presets.py)
endif()

ament_package()
