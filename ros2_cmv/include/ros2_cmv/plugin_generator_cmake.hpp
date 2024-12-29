#ifndef PLUGIN_GENERATOR_CMAKE_HPP_
#define PLUGIN_GENERATOR_CMAKE_HPP_

#include "ros2_cmv/generators/generator.hpp"

namespace ros2_cmv
{
    class PluginGeneratorCMake
    {
    public:
        PluginGeneratorCMake();

        ~PluginGeneratorCMake();

        std::string formatMessagePath(const std::string &projectName, const std::string &msgPath);

        void generatePlugin(std::string &msgFilePath, std::string &projectName,
                            std::string &relativeMsgPath, std::string &outputCorePath);
    };
};

#endif