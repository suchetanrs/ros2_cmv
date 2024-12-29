#ifndef CMAKELISTS_GENERATOR_HPP_
#define CMAKELISTS_GENERATOR_HPP_

#include "ros2_cmv/cmv_common.hpp"

namespace ros2_cmv
{
    // Function to generate CMakeLists.txt...Replaces ${PROJECT_NAME} within the CMakeLists.txt
    // project_name: To replace in cmakelists.txt
    // input_file: The input base file to read from
    // output_file: The output file to write to
    // additional_package: The additional package to add to the CMakeLists.txt
    void generateCMakeLists(const std::string &project_name, const std::string &input_file,
                            const std::string &output_file, std::string additional_package);
};

#endif