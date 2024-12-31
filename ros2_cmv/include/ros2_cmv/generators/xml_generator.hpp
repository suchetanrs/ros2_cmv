#ifndef PACKAGE_XML_GENERATOR_HPP_
#define PACKAGE_XML_GENERATOR_HPP_

#include "ros2_cmv/cmv_common.hpp"
#include "ros2_cmv/cmv_macros.hpp"

namespace ros2_cmv
{
    // Function to generate rviz_plugin.xml
    void generatePluginXML(const std::string &messageName, const std::string &pluginName,
                           const std::string &output_file, const std::string &project_name);

    // Function to generate package.xml
    void generatePackageXML(const std::string &new_name, const std::string &input_file,
                            const std::string &output_file, std::string additional_package);
};
#endif