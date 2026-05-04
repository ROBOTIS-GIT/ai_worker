cmake_minimum_required(VERSION 3.16)
project(collision_avoidance LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)
find_package(urdf REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree filters)

add_executable(voxel_create
  src/voxel_create.cpp
)


target_include_directories(voxel_create PRIVATE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(voxel_create
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  message_filters
  urdf
)

target_link_libraries(voxel_create ${PCL_LIBRARIES})

install(TARGETS
  voxel_create
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
