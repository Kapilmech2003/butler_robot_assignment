cmake_minimum_required(VERSION 3.8)
project(butler_navigation_cpp)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(butler_navigation src/butler_navigation.cpp)

ament_target_dependencies(
  butler_navigation
  rclcpp
  rclcpp_action
  nav2_msgs
)

install(TARGETS
  butler_navigation
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
