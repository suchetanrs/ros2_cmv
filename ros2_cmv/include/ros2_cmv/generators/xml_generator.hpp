#ifndef PACKAGE_XML_GENERATOR_HPP_
#define PACKAGE_XML_GENERATOR_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>
#include "ros2_cmv/common_values.hpp"

namespace ros2_cmv
{
    // Function to generate rviz_plugin.xml
    void generatePluginXML(const std::string &libpath, const std::string &pluginName,
                           const std::string &output_file, const std::string &project_name);

    // Function to generate package.xml
    void generatePackageXML(const std::string &new_name, const std::string &input_file,
                            const std::string &output_file, std::string additional_package);
};
#endif