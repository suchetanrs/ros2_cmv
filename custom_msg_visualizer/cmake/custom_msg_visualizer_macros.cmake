# Copyright 2025 Suchetan Saravanan.
# 
# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at

#   http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
# KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.   

function(GetFileNameWithoutExtension path output_var)
    get_filename_component(file_name ${path} NAME)

    get_filename_component(extension ${file_name} EXT)
    if(NOT ${extension} STREQUAL ".msg")
        message(FATAL_ERROR "Error: The file '${file_name}' does not have a '.msg' extension.")
    endif()

    get_filename_component(base_name ${file_name} NAME_WE)

    set(${output_var} ${base_name} PARENT_SCOPE)
endfunction()

macro(generate_rviz_plugin)
    string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO)
    add_definitions(-DROS_DISTRO_${ROS_DISTRO})
    # ################################################ COMMON #######################################
    find_package(ament_cmake REQUIRED)
    find_package(ament_cmake_ros REQUIRED)
    find_package(pluginlib REQUIRED)
    find_package(rviz_common REQUIRED)
    find_package(rviz_rendering REQUIRED)
    find_package(rviz_default_plugins REQUIRED)
    find_package(custom_msg_visualizer_msgs REQUIRED)
    find_package(rosidl_runtime_cpp REQUIRED)
    find_package(rosidl_typesupport_cpp REQUIRED)
    find_package(rosidl_typesupport_interface REQUIRED)
    find_package(Qt5 COMPONENTS Widgets REQUIRED)

    set(dependencies
        pluginlib
        rviz_common
        rviz_rendering
        rviz_default_plugins
        custom_msg_visualizer
        custom_msg_visualizer_msgs
        rosidl_runtime_cpp
        rosidl_typesupport_cpp
        rosidl_typesupport_interface
    )

    set(_plugin_generator_cmake_path "${custom_msg_visualizer_DIR}/../../../lib/custom_msg_visualizer/plugin_generator_cmake")

    message(STATUS "Current list dir: ${CMAKE_CURRENT_LIST_DIR}")
    message(STATUS "Cmake install prefix: ${CMAKE_INSTALL_PREFIX}")
    message(STATUS "ROS 2 CMV plugin_generator dir: ${custom_msg_visualizer_DIR}/../../../lib/custom_msg_visualizer/plugin_generator_cmake")

    # set(_plugin_generator_cmake_path "$<TARGET_FILE:plugin_generator_cmake>")
    message(STATUS "Found plugin_generator_cmake at ${_plugin_generator_cmake_path}")

    if(NOT _plugin_generator_cmake_path)
        message(FATAL_ERROR "Could not find plugin_generator_cmake target!")
    endif()

    include_directories(${rviz_default_plugins_INCLUDE_DIRS})


    foreach(_message_file ${ARGV})
        # ######################################### GENERATE FILES #########################################

        message(STATUS "Running plugin_generator_cmake from custom_msg_visualizer...")
        GetFileNameWithoutExtension(${_message_file} _message_name)

        execute_process(
            COMMAND "${_plugin_generator_cmake_path}" ${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR} ${_message_file}
            WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
            RESULT_VARIABLE _ret
            ERROR_VARIABLE _error
        )

        if(_ret)
            message(WARNING "plugin_generator_cmake failed with return code: ${_ret} \n Error: ${_error}\nSkipping plugin generation for message ${_message_file}.")
            continue()
        endif()

        ############################################## BUILD FILES ###########################################
        qt5_wrap_cpp(MOC_FILES_${_message_name}
            include/${PROJECT_NAME}/message_specific/${_message_name}/custom_msg_display.hpp
        )

        foreach(moc_file ${MOC_FILES_${_message_name}})
            message(STATUS "MOC file ${_message_name}: ${moc_file}")
        endforeach()


        add_library(${PROJECT_NAME}_${_message_name}_rviz_plugin
            src/message_specific/${_message_name}/custom_msg_process.cpp
            src/message_specific/${_message_name}/custom_msg_display.cpp
            src/message_specific/${_message_name}/array_assist.cpp
            ${_generated_headers}
            ${MOC_FILES_${_message_name}}
        )

        target_include_directories(${PROJECT_NAME}_${_message_name}_rviz_plugin
            PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/rosidl_generator_cpp>
            $<INSTALL_INTERFACE:include>
        )

        ament_target_dependencies(${PROJECT_NAME}_${_message_name}_rviz_plugin
            ${dependencies}
        )

        target_link_libraries(${PROJECT_NAME}_${_message_name}_rviz_plugin
            rviz_default_plugins::rviz_default_plugins
            ${PROJECT_NAME}__rosidl_typesupport_cpp
        )
        target_compile_definitions(${PROJECT_NAME}_${_message_name}_rviz_plugin PRIVATE PROJECT_NAME=${PROJECT_NAME})
        target_compile_definitions(${PROJECT_NAME}_${_message_name}_rviz_plugin PRIVATE MESSAGE_NAME=${_message_name})

        install(
            TARGETS ${PROJECT_NAME}_${_message_name}_rviz_plugin
            EXPORT export_${PROJECT_NAME}_${_message_name}_rviz_plugin
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
        )

        ament_export_targets(export_${PROJECT_NAME}_${_message_name}_rviz_plugin)
        pluginlib_export_plugin_description_file(rviz_common plugin_${_message_name}.xml)
    endforeach()

    install(
        DIRECTORY include/
        DESTINATION include
    )

    ament_export_include_directories(include)

endmacro()
