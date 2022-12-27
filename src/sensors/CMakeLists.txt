cmake_minimum_required(VERSION 3.8)
project(sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

find_package(JetsonGPIO REQUIRED)

include_directories(include)

# add_library(wheel_encoder SHARED src/wheel_encoder/wheel_encoder_publisher.cc)
# target_compile_definitions(wheel_encoder PRIVATE WAIT_SET_DLL)
# ament_target_dependencies(wheel_encoder rclcpp rclcpp_components std_msgs)
# rclcpp_components_register_node(
#     wheel_encoder
#     PLUGIN "WheelEncoder"
#     EXECUTABLE wheel_encoder_publisher)

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

add_executable(wheel_encoder_test src/wheel_encoder/wheel_encoder_publisher.cc)
ament_target_dependencies(wheel_encoder_test rclcpp std_msgs JetsonGPIO)

add_executable(motor_test src/motor_controller/motor_controller.cc)
ament_target_dependencies(wheel_encoder_test rclcpp std_msgs JetsonGPIO)

install(TARGETS
  wheel_encoder_test
  DESTINATION lib/${PROJECT_NAME})

ament_package()
