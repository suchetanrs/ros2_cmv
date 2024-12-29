#
# A simple macro that executes the installed plugin_generator_cmake.
#
macro(generate_rviz_plugin)
    # ######################################### GENERATE FILES #########################################
    set(_plugin_generator_cmake_path "${ros2_cmv_DIR}/../../../lib/ros2_cmv/plugin_generator_cmake")
    
    message(STATUS "Current list dir: ${CMAKE_CURRENT_LIST_DIR}")
    message(STATUS "Cmake install prefix: ${CMAKE_INSTALL_PREFIX}")
    message(STATUS "ROS 2 CMV plugin_generator dir: ${ros2_cmv_DIR}/../../../lib/ros2_cmv/plugin_generator_cmake")

    # set(_plugin_generator_cmake_path "$<TARGET_FILE:plugin_generator_cmake>")
    message(STATUS "Found plugin_generator_cmake at ${_plugin_generator_cmake_path}")
    
    if(NOT _plugin_generator_cmake_path)
        message(FATAL_ERROR "Could not find plugin_generator_cmake target!")
    endif()

    message(STATUS "Running plugin_generator_cmake from ros2_cmv...")

    execute_process(
        COMMAND "${_plugin_generator_cmake_path}" ${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR} ${ARGV}
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        RESULT_VARIABLE _ret
    )

    if(_ret)
        message(FATAL_ERROR "plugin_generator_cmake failed with return code ${_ret}")
    endif()

    ############################################## BUILD FILES ###########################################
    find_package(ament_cmake REQUIRED)
    find_package(ament_cmake_ros REQUIRED)
    find_package(pluginlib REQUIRED)
    find_package(rviz_common REQUIRED)
    find_package(rviz_rendering REQUIRED)
    find_package(rviz_default_plugins REQUIRED)
    find_package(ros2_cmv REQUIRED)
    find_package(ros2_cmv_example REQUIRED)
    find_package(rosidl_runtime_cpp REQUIRED)
    find_package(rosidl_typesupport_cpp REQUIRED)
    find_package(rosidl_typesupport_interface REQUIRED)
    find_package(Qt5 COMPONENTS Widgets REQUIRED)

    set(dependencies
        pluginlib
        rviz_common
        rviz_rendering
        rviz_default_plugins
        ros2_cmv
        ros2_cmv_example
        rosidl_runtime_cpp
        rosidl_typesupport_cpp
        rosidl_typesupport_interface
    )

    set(CMAKE_AUTOMOC ON)
    qt5_wrap_cpp(MOC_FILES
    include/${PROJECT_NAME}/custom_msg_display.hpp
    )

    include_directories(${rviz_default_plugins_INCLUDE_DIRS})

    # message("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    # message("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    # message("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    # message("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    # message("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    # foreach(_non_idl_file ${_non_idl_files})
    #     message("Non IDL file: ${_non_idl_file}")
    # endforeach()
    # foreach(_idl_file ${_idl_files})
    #     message("IDL file: ${_idl_file}")
    # endforeach()
    # foreach(_dep_file ${_dep_files})
    #     message("Dep file: ${_dep_file}")
    # endforeach()
    # foreach(mocfile ${MOC_FILES})
    #     message("MOC file: ${mocfile}")
    # endforeach()
    # foreach(header ${_generated_headers})
    #     message("Header: ${header}")
    # endforeach()
    

    add_library(${PROJECT_NAME}_rviz_plugin
    src/custom_msg_display.cpp
    src/custom_msg_process.cpp
    ${MOC_FILES}
    ${_generated_headers}
    )

    target_include_directories(${PROJECT_NAME}_rviz_plugin
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/rosidl_generator_cpp>
    $<INSTALL_INTERFACE:include>
    )

    ament_target_dependencies(${PROJECT_NAME}_rviz_plugin
    ${dependencies}
    )

    target_link_libraries(${PROJECT_NAME}_rviz_plugin 
    rviz_default_plugins::rviz_default_plugins 
    ${PROJECT_NAME}__rosidl_typesupport_cpp
    )
    target_compile_definitions(${PROJECT_NAME}_rviz_plugin PRIVATE PROJECT_NAME=${PROJECT_NAME})

    install(
    TARGETS ${PROJECT_NAME}_rviz_plugin
    EXPORT export_${PROJECT_NAME}_rviz_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    )

    install(
    DIRECTORY include/
    DESTINATION include
    )

    ament_export_include_directories(include)
    ament_export_targets(export_${PROJECT_NAME}_rviz_plugin)
    pluginlib_export_plugin_description_file(rviz_common plugin.xml)

endmacro()