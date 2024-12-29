#include "ros2_cmv/cmv_common.hpp"

namespace ros2_cmv
{
    // Function to trim whitespace from both ends of a string
    std::string trim(const std::string &s)
    {
        size_t start = s.find_first_not_of(" \t\r\n");
        if (start == std::string::npos)
            return "";
        size_t end = s.find_last_not_of(" \t\r\n");
        return s.substr(start, end - start + 1);
    }

    // Function to split a string by whitespace
    std::vector<std::string> split(const std::string &s)
    {
        std::vector<std::string> tokens;
        std::istringstream iss(s);
        std::string token;
        while (iss >> token)
        {
            tokens.push_back(token);
        }
        return tokens;
    }
    
    std::string convertRosTypeToCpp(const std::string &ros_type)
    {
        size_t slash_pos = ros_type.find('/');
        if (slash_pos == std::string::npos)
        {
            RCLCPP_ERROR_STREAM(globalValues.getLogger(), "Invalid ROS type format: " << ros_type);
            throw std::runtime_error("Invalid ROS type format: " + ros_type);
        }
        std::string package = ros_type.substr(0, slash_pos);
        std::string message = ros_type.substr(slash_pos + 1);
        return package + "::msg::" + message;
    }

    std::string convertCamelCaseToSnakeCase(const std::string &input)
    {
        std::string result = input;

        // First substitution
        std::regex first_pattern(R"((.)([A-Z][a-z]+))");
        result = std::regex_replace(result, first_pattern, "$1_$2");

        // Second substitution
        std::regex second_pattern(R"(([a-z0-9])([A-Z]))");
        result = std::regex_replace(result, second_pattern, "$1_$2");

        // Convert to lowercase
        std::transform(result.begin(), result.end(), result.begin(), ::tolower);

        return result;
    }

    std::string convertToIncludePath(const std::string &input)
    {
        // Find the position of the last '/'
        size_t pos = input.rfind('/');
        if (pos == std::string::npos)
        {
            throw std::runtime_error("Invalid input: No '/' found in the input.");
        }

        // Split the input into prefix and class name
        std::string prefix = input.substr(0, pos);
        std::string className = input.substr(pos + 1);

        // Convert the class name to snake_case and append ".hpp"
        std::string snakeCaseName = convertCamelCaseToSnakeCase(className) + ".hpp";

        // Construct the include path
        return prefix + "/msg/" + snakeCaseName;
    }

    // Function to extract the package name from the given input
    std::string convertToPackageName(const std::string &input)
    {
        // Find the position of the last '/'
        size_t pos = input.rfind('/');

        // Extract and return the package name (substring before the last '/')
        return input.substr(0, pos);
    }

    std::string convertToRvizPluginName(const std::string &input)
    {
        // Find the slash
        std::size_t slashPos = input.find('/');
        if (slashPos == std::string::npos)
        {
            throw std::runtime_error("Invalid input: No '/' found in the input.");
        }

        // Extract the parts before and after the slash
        std::string packagePart = input.substr(0, slashPos);
        std::string classPart = input.substr(slashPos + 1);

        // Convert the classPart to lowercase
        std::string classPartLower = classPart;
        std::transform(
            classPartLower.begin(),
            classPartLower.end(),
            classPartLower.begin(),
            ::tolower);

        // Construct the final string
        // e.g. "ros2_cmv_example_example_rviz_plugin"
        return "rviz_plugin_" + packagePart + "_" + classPartLower;
        // return packagePart + "_" + classPartLower + "_rviz_plugin";
    }
};