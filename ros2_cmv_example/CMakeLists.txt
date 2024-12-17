cmake_minimum_required(VERSION 3.5)
project(ros2_cmv_example)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  include
)

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/Example.msg"
DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs sensor_msgs visualization_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()