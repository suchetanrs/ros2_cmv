cmake_minimum_required(VERSION 3.8)
project(custom_msg_visualizer)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g1")
string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO)
add_definitions(-DROS_DISTRO_${ROS_DISTRO})

## --------------- FIND PACKAGES --------------- ##
## --------------------------------------------- ##
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(custom_msg_visualizer_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)


## --------------- GENERIC --------------- ##
## --------------------------------------- ##

set(CMAKE_AUTOMOC ON)
set(MESSAGE_NAME "AllTypes")
include_directories(${rviz_default_plugins_INCLUDE_DIRS})

## --------------- CMV_COMMON --------------- ##
## -------------------------------------------------- ##

add_library(cmv_common
  src/cmv_common.cpp
)

target_compile_definitions(cmv_common PRIVATE PROJECT_NAME=${PROJECT_NAME} MESSAGE_NAME=${MESSAGE_NAME})

target_include_directories(cmv_common
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cmv_common
  rviz_common
)

install(
  TARGETS cmv_common
  EXPORT export_cmv_common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_cmv_common)

## --------------- CUSTOM_MSG_DISPLAY --------------- ##
## -------------------------------------------------- ##

qt5_wrap_cpp(CUSTOM_MSG_MOC_FILES
  include/custom_msg_visualizer/message_specific/${MESSAGE_NAME}/custom_msg_display.hpp
)

add_library(custom_msg_display
  src/message_specific/${MESSAGE_NAME}/custom_msg_process.cpp
  src/message_specific/${MESSAGE_NAME}/custom_msg_display.cpp
  src/array_assist.cpp
  ${CUSTOM_MSG_MOC_FILES}
)

target_compile_definitions(custom_msg_display PRIVATE PROJECT_NAME=${PROJECT_NAME} MESSAGE_NAME=${MESSAGE_NAME})

target_include_directories(custom_msg_display
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(custom_msg_display
  rviz_common
  pluginlib
  rviz_rendering
  rviz_default_plugins
  custom_msg_visualizer_msgs
)
target_link_libraries(custom_msg_display cmv_common rviz_default_plugins::rviz_default_plugins Qt5::Widgets)

install(
  TARGETS custom_msg_display
  EXPORT export_custom_msg_display
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_custom_msg_display)

# ## --------------- GENERATORS --------------- ##
# ## ---------------------------------------------------- ##

add_library(cmv_generator
  src/generators/cmakelists_generator.cpp
  src/generators/xml_generator.cpp
  src/generators/hpp_cpp_generator.cpp
  src/generators/generator.cpp
  src/ament_index_helpers.cpp
)

target_compile_definitions(cmv_generator PRIVATE PROJECT_NAME=${PROJECT_NAME} MESSAGE_NAME=${MESSAGE_NAME})

target_include_directories(cmv_generator
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cmv_generator
  rviz_common
  pluginlib
  custom_msg_visualizer_msgs
  ament_index_cpp
)
target_link_libraries(cmv_generator cmv_common)

install(
  TARGETS cmv_generator
  EXPORT export_cmv_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_cmv_generator)

# ## --------------- PLUGIN_GENERATOR GUI --------------- ##
# ## ---------------------------------------------------- ##

qt5_wrap_cpp(GENERATOR_GUI_MOC_FILES
  include/custom_msg_visualizer/plugin_generator_app.hpp
)

add_executable(plugin_generator_app
  src/plugin_generator_app.cpp
  ${GENERATOR_GUI_MOC_FILES}
)

target_compile_definitions(plugin_generator_app PRIVATE PROJECT_NAME=${PROJECT_NAME} MESSAGE_NAME=${MESSAGE_NAME})

target_include_directories(plugin_generator_app
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(plugin_generator_app
  rviz_common
  pluginlib
  custom_msg_visualizer_msgs
  ament_index_cpp
)

target_link_libraries(plugin_generator_app Qt5::Widgets cmv_generator)

install(TARGETS
  plugin_generator_app
  DESTINATION lib/${PROJECT_NAME}
)

# ## --------------- PLUGIN_GENERATOR_CMAKE --------------- ##
# ## ------------------------------------------------------ ##

add_executable(plugin_generator_cmake
  src/plugin_generator_cmake.cpp
)

target_compile_definitions(plugin_generator_cmake PRIVATE PROJECT_NAME=${PROJECT_NAME} MESSAGE_NAME=${MESSAGE_NAME})

target_include_directories(plugin_generator_cmake
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(plugin_generator_cmake
  rviz_common
  pluginlib
  rviz_rendering
  rviz_default_plugins
  custom_msg_visualizer_msgs
  ament_index_cpp
)

target_link_libraries(plugin_generator_cmake cmv_generator)

install(
  TARGETS plugin_generator_cmake
  DESTINATION lib/${PROJECT_NAME}
)

## --------------- COMPLETE INSTALL --------------- ##
## ---------------------------------------------------- ##

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES base_files/base_cmakelists.txt base_files/base_package.xml base_files/custom_msg_visualizer_logo.png
  DESTINATION share/${PROJECT_NAME}/base_files
)

install(
  FILES src/message_specific/${MESSAGE_NAME}/custom_msg_display.cpp
  DESTINATION share/${PROJECT_NAME}/base_files/message_specific/${MESSAGE_NAME}
)

ament_export_include_directories(include)
pluginlib_export_plugin_description_file(rviz_common custom_msg_plugin.xml)

ament_package(
  CONFIG_EXTRAS "custom_msg_visualizer-extras.cmake"
)