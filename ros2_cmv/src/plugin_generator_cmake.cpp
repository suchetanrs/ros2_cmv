#include "ros2_cmv/plugin_generator_cmake.hpp"

namespace ros2_cmv
{
    PluginGeneratorCMake::PluginGeneratorCMake()
    {
    }

    PluginGeneratorCMake::~PluginGeneratorCMake()
    {
    }

    std::string PluginGeneratorCMake::formatMessagePath(const std::string &projectName, const std::string &msgPath)
    {
        // get the last part of the message.
        std::string msgName = std::filesystem::path(msgPath).filename().string();
        // Check if the file extension is .msg
        if (std::filesystem::path(msgName).extension() != ".msg")
        {
            throw std::runtime_error("Invalid file extension. Expected .msg while formatting message path.");
        }

        // Remove the .msg extension
        std::string baseName = std::filesystem::path(msgName).stem().string();

        // Return the formatted string
        return projectName + "/" + baseName;
    }

    void PluginGeneratorCMake::generatePlugin(std::string &msgFilePath, std::string &projectName,
                                              std::string &relativeMsgPath, std::string &outputCorePath)
    {
        std::string selectedText = formatMessagePath(projectName, relativeMsgPath);

        generateFiles(msgFilePath, outputCorePath, selectedText, projectName, false);
    }
}

int main(int argc, char *argv[])
{
    ros2_cmv::PluginGeneratorCMake pluginGeneratorCMake;

    std::cout << "**************************************************" << std::endl;
    std::cout << "**************************************************" << std::endl;
    std::cout << "**************************************************" << std::endl;
    
    std::string packageName = argv[1];
    std::string outputCorePath = argv[2];
    std::string relativeMsgPath = argv[3];

    std::string msgFilePath = outputCorePath + "/" + relativeMsgPath;

    std::cout << "packageName: " << packageName << std::endl;
    std::cout << "outputCorePath: " << outputCorePath << std::endl;
    std::cout << "relativeMsgPath: " << relativeMsgPath << std::endl;
    std::cout << "msgFilePath: " << msgFilePath << std::endl;

    try {
        pluginGeneratorCMake.generatePlugin(msgFilePath, packageName, relativeMsgPath, outputCorePath);
    } catch (const std::runtime_error& e) {
        std::cerr << e.what();
        return 1;
    } catch (const std::exception& e) {
        std::cerr << e.what();
        return 1;
    }
    return 0;
}