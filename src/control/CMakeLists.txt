cmake_minimum_required(VERSION 3.8)
project(control)

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

find_package(sensors REQUIRED)
find_package(xiaocar_msgs REQUIRED)

include_directories(
  include
  ${colcon_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME} SHARED
  src/motor/driver.cc
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

# target_link_libraries(${PROJECT_NAME} 
#   i2c
# )

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensors
  xiaocar_msgs
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# create ament index resource which references the libraries in the binary dir
# set(node_plugins "")

# add_library(wheels_driver SHARED
#   src/wheels_driver_node.cc)

# target_compile_definitions(wheels_driver
#   PRIVATE "COMPOSITION_BUILDING_DLL")

# ament_target_dependencies(wheels_driver
#   "rclcpp"
#   "rclcpp_components"
#   "std_msgs"
#   "sensors"
#   "xiaocar_msgs")

# rclcpp_components_register_nodes(wheels_driver "xiaoduckie::control::WheelsDriverNode")
# set(node_plugins "${node_plugins}xiaoduckie::control::WheelsDriverNode;$<TARGET_FILE:wheels_driver>\n")

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

add_executable(
  wheels_driver_node
  src/motor/driver.cc
  src/wheels_driver_node.cc)

ament_target_dependencies(wheels_driver_node rclcpp std_msgs xiaocar_msgs sensors)

install(
  TARGETS
  wheels_driver_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
