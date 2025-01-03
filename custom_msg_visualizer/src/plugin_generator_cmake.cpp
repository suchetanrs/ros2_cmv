/**
    Copyright 2025 Suchetan Saravanan.

    Licensed to the Apache Software Foundation (ASF) under one
    or more contributor license agreements.  See the NOTICE file
    distributed with this work for additional information
    regarding copyright ownership.  The ASF licenses this file
    to you under the Apache License, Version 2.0 (the
    "License"); you may not use this file except in compliance
    with the License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing,
    software distributed under the License is distributed on an
    "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
    KIND, either express or implied.  See the License for the
    specific language governing permissions and limitations
    under the License.
*/

#include "custom_msg_visualizer/plugin_generator_cmake.hpp"

namespace custom_msg_visualizer
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
    custom_msg_visualizer::PluginGeneratorCMake pluginGeneratorCMake;

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