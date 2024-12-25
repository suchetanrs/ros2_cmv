#ifndef PACKAGE_XML_GENERATOR_HPP_
#define PACKAGE_XML_GENERATOR_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>

namespace ros2_cmv
{
    // Function to generate rviz_plugin.xml
    void generatePluginXML(const std::string &libpath, const std::string &pluginName, const std::string &output_file, const std::string &project_name);

    void generatePackageXML(const std::string &new_name, const std::string &input_file, const std::string &output_file, std::string additional_package);
};
#endif