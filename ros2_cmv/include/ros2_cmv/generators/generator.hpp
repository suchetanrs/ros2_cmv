#ifndef GENERATOR_HPP_
#define GENERATOR_HPP_

#include "ros2_cmv/generators/hpp_cpp_generator.hpp"
#include "ros2_cmv/generators/xml_generator.hpp"
#include "ros2_cmv/generators/cmakelists_generator.hpp"

#include "ros2_cmv/ament_index_helpers.hpp"

namespace ros2_cmv
{
    // Function to check if a file is a valid ROS message type for generation
    bool checkMessageValidity(const std::string &msg_file_path, std::vector<int> &valid_line_idx);

    void generateFiles(std::string &msgFilePath, std::string &outputPackageDir,
                       std::string &selectedMessage, std::string &projectName,
                       bool newPackage);
};

#endif