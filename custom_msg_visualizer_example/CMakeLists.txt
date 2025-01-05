cmake_minimum_required(VERSION 3.5)
project(custom_msg_visualizer_example)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(custom_msg_visualizer REQUIRED)

include_directories(
  include
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExampleCMake.msg"
  "msg/ExampleCMake2.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs sensor_msgs visualization_msgs
)

# generate_rviz_plugin(
#   "msg/ExampleCMake.msg"
#   "msg/ExampleCMake2.msg"
# )

ament_export_dependencies(rosidl_default_runtime)

ament_package()