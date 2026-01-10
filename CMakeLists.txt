cmake_minimum_required(VERSION 3.8)
project(motor_health_monitor)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

include_directories(include)

add_library(motor_health_component SHARED
  src/cmd_odom_sync.cpp
  src/motor_health_node.cpp
)

ament_target_dependencies(motor_health_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  lifecycle_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  diagnostic_msgs
)

rclcpp_components_register_nodes(motor_health_component
  "motor_health_monitor::MotorHealthNode")

install(TARGETS motor_health_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include launch config scripts
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Test for CmdOdomSync
  ament_add_gtest(test_cmd_odom_sync test/test_cmd_odom_sync.cpp)
  target_link_libraries(test_cmd_odom_sync motor_health_component)
  target_include_directories(test_cmd_odom_sync PUBLIC include)

  # Test for MotorFaultDetector
  ament_add_gtest(test_motor_fault_detector test/test_motor_fault_detector.cpp)
  target_link_libraries(test_motor_fault_detector motor_health_component)
  target_include_directories(test_motor_fault_detector PUBLIC include)

  # Test for MotorControllerFactory
  ament_add_gtest(test_motor_controller_factory test/test_motor_controller_factory.cpp)
  target_link_libraries(test_motor_controller_factory motor_health_component)
  target_include_directories(test_motor_controller_factory PUBLIC include)
endif()

ament_package()
