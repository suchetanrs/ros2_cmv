/**
    Copyright 2025 Suchetan Saravanan.

    Licensed to the Apache Software Foundation (ASF) under one
    or more contributor license agreements.  See the NOTICE file
    distributed with this work for additional information
    regarding copyright ownership.  The ASF licenses this file
    to you under the Apache License, Version 2.0 (the
    "License"); you may not use this file except in compliance
    with the License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing,
    software distributed under the License is distributed on an
    "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
    KIND, either express or implied.  See the License for the
    specific language governing permissions and limitations
    under the License.
*/

#ifndef CMV_COMMON_HPP_
#define CMV_COMMON_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <sstream>
#include <filesystem>
#include <regex>

namespace custom_msg_visualizer
{
    class GlobalValues
    {
    private:
        GlobalValues() : logger(rclcpp::get_logger("plugin_generator")) {};
        std_msgs::msg::Header header;
        rclcpp::Logger logger;

        // Delete copy constructor and assignment operator to prevent copying
        GlobalValues(const GlobalValues &) = delete;
        GlobalValues &operator=(const GlobalValues &) = delete;

    public:
        static GlobalValues &getInstance()
        {
            static GlobalValues instance;
            return instance;
        }

        void setHeader(const std_msgs::msg::Header &new_header)
        {
            header = new_header;
        }

        const std_msgs::msg::Header &getHeader() const
        {
            return header;
        }

        rclcpp::Logger &getLogger()
        {
            return logger;
        }
    };

    inline GlobalValues &globalValues = GlobalValues::getInstance();

    // --------------------------------------------------------------------------
    // GENERAL PURPOSE STRUCTS

    struct Message
    {
        std::string type;
        std::string name;
    };

    // ---------------------------------------------------------------------------
    // STRING HELPERS

    // Function to trim whitespace from both ends of a string
    std::string trim(const std::string &s);

    // Function to split a string by whitespace
    std::vector<std::string> split(const std::string &s);

    // ---------------------------------------------------------------------------
    // CONVERSIONS FUNCTIONS

    // Function to convert ROS message type to C++ type
    // custom_msg_visualizer_msgs/AllTypes -> custom_msg_visualizer_msgs::msg::AllTypes
    std::string convertRosTypeToCpp(const std::string &ros_type);

    // custom_msg_visualizer_all_types/AllTypes -> AllTypes
    std::string convertToMessageName(const std::string &ros_type);

    // Function to convert camel case to snake case
    // AllTypes -> all_types
    // Conversion logic ported to CPP from rosidl
    std::string convertCamelCaseToSnakeCase(const std::string &input);

    // "custom_msg_visualizer_msgs/AllTypes" -> "custom_msg_visualizer_msgs/msg/all_types.hpp"
    std::string convertToIncludePath(const std::string &input);

    // Function to convert "custom_msg_visualizer_msgs/AllTypes" -> "custom_msg_visualizer_msgs"
    std::string convertToPackageName(const std::string &input);

    // Function to convert "custom_msg_visualizer_msgs/AllTypes" -> "custom_msg_visualizer_msgs_all_types"
    std::string convertToRvizPluginName(const std::string &input);
};

#endif