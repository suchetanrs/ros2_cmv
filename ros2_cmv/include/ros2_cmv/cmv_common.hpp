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

namespace ros2_cmv
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
    // ros2_cmv_example/Example -> ros2_cmv_example::msg::Example
    std::string convertRosTypeToCpp(const std::string &ros_type);

    // Function to convert camel case to snake case
    // Example -> example
    // Conversion logic ported to CPP from rosidl
    std::string convertCamelCaseToSnakeCase(const std::string &input);

    // "ros2_cmv_example/Example" -> "ros2_cmv_example/msg/example.hpp"
    std::string convertToIncludePath(const std::string &input);

    // Function to convert "ros2_cmv_example/Example" -> "ros2_cmv_example"
    std::string convertToPackageName(const std::string &input);

    // Function to convert "ros2_cmv_example/Example" -> "ros2_cmv_example_example"
    std::string convertToRvizPluginName(const std::string &input);
};

#endif