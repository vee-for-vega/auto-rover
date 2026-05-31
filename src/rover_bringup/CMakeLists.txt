cmake_minimum_required(VERSION 3.8)
project(rover_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(odom_publisher src/odom_publisher.cpp)
ament_target_dependencies(odom_publisher rclcpp nav_msgs geometry_msgs std_msgs)

add_executable(odom_subscriber src/odom_subscriber.cpp)
ament_target_dependencies(odom_subscriber rclcpp nav_msgs)

add_executable(safety_status_service src/safety_status_service.cpp)
ament_target_dependencies(safety_status_service rclcpp std_srvs std_msgs)

install(TARGETS
  odom_publisher
  odom_subscriber
  safety_status_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()