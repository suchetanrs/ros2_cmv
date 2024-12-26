#ifndef HEADER_GENERATOR_HPP_
#define HEADER_GENERATOR_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <sstream>
#include <filesystem>
#include <regex>

#include "ros2_cmv/custom_msg_metadata.hpp"
#include "ros2_cmv/common_values.hpp"

namespace ros2_cmv
{
    struct Message
    {
        std::string type;
        std::string name;
    };

    // Function to trim whitespace from both ends of a string
    std::string trim(const std::string &s);

    // Function to split a string by whitespace
    std::vector<std::string> split(const std::string &s);

    // Function to check if a file is a valid ROS message type for generation
    bool check_message_validity(const std::string &msg_file_path, std::vector<int> &valid_line_idx);

    // Function to parse a .msg file and return a vector of Message structs
    std::vector<Message> parse_msg_file(const std::string &msg_file_path, std::vector<int> &valid_line_idx);

    // Function to generate custom_msg_metadata.hpp file.
    void generate_cpp_header(const std::string &MSG_FILE, const std::string &output_file,
                             const std::string &CUSTOM_MESSAGE_HEADER, const std::string &CUSTOM_MESSAGE_TYPE,
                             std::vector<int> &valid_line_idx);

    // Function to copy a file from input_path to output_path
    void copyFile(const std::string &input_path, const std::string &output_path);

    // CONVERSIONS FUNCTIONS

    // Function to convert ROS message type to C++ type
    std::string convert_ros_type_to_cpp(const std::string &ros_type);

    // Function to convert camel case to snake case
    std::string convertCamelCaseToSnakeCase(const std::string &input);

    // Function to convert "ros2_cmv_example/Example" to "ros2_cmv_example/msg/example.hpp"
    std::string convertToIncludePath(const std::string &input);

    // Function to convert "ros2_cmv_example/Example" to "ros2_cmv_example::msg::Example"
    std::string convertToNamespace(const std::string &input);

    // Function to convert "ros2_cmv_example/Example" to "ros2_cmv_example"
    std::string convertToPackageName(const std::string &input);

    // Function to convert "ros2_cmv_example/Example" to "ros2_cmv_example_example"
    std::string convertToRvizPluginName(const std::string &input);
};

#endif