#ifndef CMAKELISTS_GENERATOR_HPP_
#define CMAKELISTS_GENERATOR_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "ros2_cmv/common_values.hpp"

namespace ros2_cmv
{
    // Function to generate CMakeLists.txt...Replaces ${PROJECT_NAME} within the CMakeLists.txt
    void generateCMakeLists(const std::string &project_name, const std::string &input_file,
                            const std::string &output_file, std::string additional_package);
};

#endif