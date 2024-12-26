cmake_minimum_required(VERSION 3.8)
project(ros2_cmv)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(ros2_cmv_example REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_c REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/ros2_cmv/custom_msg_display.hpp
)

qt5_wrap_cpp(MOC_FILES2
  include/ros2_cmv/pluginGeneratorApp.hpp
)

include_directories(${rviz_default_plugins_INCLUDE_DIRS})

add_library(custom_msg_display
  src/custom_msg_display.cpp
  ${MOC_FILES}
)

add_executable(msg_loader
  src/custom_msg_display.cpp
  src/generators/header_generator.cpp
  src/generators/xml_generator.cpp
  src/generators/cmakelists_generator.cpp
  src/pluginGeneratorApp.cpp
  ${MOC_FILES}
  ${MOC_FILES2}
)

target_compile_definitions(custom_msg_display PRIVATE PROJECT_NAME=${PROJECT_NAME})
target_compile_definitions(msg_loader PRIVATE PROJECT_NAME=${PROJECT_NAME})

target_include_directories(custom_msg_display
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(msg_loader
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(custom_msg_display
  rviz_common
  pluginlib
  rviz_rendering
  rviz_default_plugins
  ros2_cmv_example
)

ament_target_dependencies(msg_loader
  rviz_common
  pluginlib
  rviz_rendering
  rviz_default_plugins
  ros2_cmv_example
  ament_index_cpp
)

target_link_libraries(custom_msg_display rviz_default_plugins::rviz_default_plugins Qt5::Widgets)
target_link_libraries(msg_loader Qt5::Widgets)

install(TARGETS
  msg_loader
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS custom_msg_display
  EXPORT export_custom_msg_display
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES base_files/base_cmakelists.txt base_files/base_package.xml base_files/ros2_cmv_logo.jpg src/custom_msg_display.cpp
  DESTINATION share/${PROJECT_NAME}/base_files
)

ament_export_include_directories(include)
ament_export_targets(export_custom_msg_display)
pluginlib_export_plugin_description_file(rviz_common custom_msg_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
