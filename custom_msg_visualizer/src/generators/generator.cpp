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

#include "custom_msg_visualizer/generators/generator.hpp"

namespace custom_msg_visualizer
{
    bool checkMessageValidity(const std::string &msg_file_path, std::vector<int> &valid_line_idx)
    {
        bool headerExists = false;
        std::ifstream file(msg_file_path);
        if (!file.is_open())
        {
            std::cerr << "Error: Unable to open " << msg_file_path;
            throw std::runtime_error("Error: Unable to open " + msg_file_path);
        }
        std::string line;
        int currIdx = -1;
        while (std::getline(file, line))
        {
            currIdx++;
            // find the first # and remove everything after it
            size_t comment_pos = line.find('#');
            if (comment_pos != std::string::npos)
            {
                line = line.substr(0, comment_pos);
            }

            // Trim whitespace
            line = trim(line);
            if (line.empty())
            {
                continue; // Skip empty lines
            }

            // Split line into parts using whitespace
            std::vector<std::string> parts = split(line);
            if (parts.size() == 3 || parts.size() == 2)
            {
                std::string msg_type = parts[0];
                std::string var_name = parts[1];
                // std::string default_value = parts[2]; // Not used in C++ generation
                if (msg_type == "std_msgs/Header")
                {
                    valid_line_idx.push_back(currIdx);
                    headerExists = true;
                    continue;
                }
                if (std::find(variableTypes.begin(), variableTypes.end(), msg_type) != variableTypes.end())
                {
                    valid_line_idx.push_back(currIdx);
                }
            }
        }
        file.close();
        if (!headerExists || valid_line_idx.size() == 0)
            return false;
        return true;
    }

    void generateFiles(std::string &msgFilePath, std::string &outputPackageDir,
                       std::string &selectedMessage, std::string &projectName,
                       bool newPackage)
    {

        std::vector<std::string> directories = {
            outputPackageDir + "/src/message_specific/" + convertToMessageName(selectedMessage) + "/",
            outputPackageDir + "/include/" + projectName + "/message_specific/" + convertToMessageName(selectedMessage) + "/"};

        // Iterate through each directory and create it if it doesn't exist
        for (const auto &directory : directories)
        {
            try
            {
                if (std::filesystem::create_directories(directory))
                {
                    std::cout << "Directory created: " << directory << std::endl;
                }
                else
                {
                    std::cout << "Directory already exists: " << directory << std::endl;
                }
            }
            catch (const std::filesystem::filesystem_error &e)
            {
                throw std::runtime_error("Failed to create directory: " + directory);
            }
        }

        std::vector<int> validIdx;
        if (!checkMessageValidity(msgFilePath, validIdx))
        {
            throw std::runtime_error("Message " + selectedMessage + " is not valid. It does not have a header or has 0 valid lines.");
        }

        // Parse the .msg file
        std::vector<Message> messages = parseMsgFile(msgFilePath, validIdx);
        if (messages.empty())
        {
            std::cerr << "No valid messages found in the .msg file.";
            // throw std::runtime_error("No valid messages found in the .msg file.");
        }

        std::cout << "DISPLAY HEADER FILE===================================================================" << std::endl;
        copyFile(getPackagePrefix("custom_msg_visualizer") + "/include/custom_msg_visualizer/message_specific/AllTypes/custom_msg_display.hpp", outputPackageDir + "/include/" + projectName + "/message_specific/" + convertToMessageName(selectedMessage) + "/custom_msg_display.hpp");
        std::cout << "OTHER HEADER FILES===================================================================" << std::endl;
        generateMetadataHeader(messages, outputPackageDir + "/include/" + projectName + "/message_specific/" + convertToMessageName(selectedMessage) + "/custom_msg_metadata.hpp", convertToIncludePath(selectedMessage), convertRosTypeToCpp(selectedMessage));
        copyFile(getPackagePrefix("custom_msg_visualizer") + "/include/custom_msg_visualizer/message_specific/AllTypes/custom_msg_process.hpp", outputPackageDir + "/include/" + projectName + "/message_specific/" + convertToMessageName(selectedMessage) + "/custom_msg_process.hpp");
        std::cout << "CPP FILES===================================================================" << std::endl;
        copyFile(getPackagePrefix("custom_msg_visualizer") + "/share/custom_msg_visualizer/base_files/message_specific/AllTypes/custom_msg_display.cpp", outputPackageDir + "/src/message_specific/" + convertToMessageName(selectedMessage) + "/custom_msg_display.cpp");
        generateProcessMsgFile(messages, outputPackageDir + "/src/message_specific/" + convertToMessageName(selectedMessage) + "/custom_msg_process.cpp");
        std::cout << "PLUGIN XML===================================================================" << std::endl;
        generatePluginXML(convertToMessageName(selectedMessage), convertToMessageName(selectedMessage), outputPackageDir + "/plugin_" + convertToMessageName(selectedMessage) + ".xml", projectName);
        if (newPackage)
        {
            std::cout << "CMAKELISTS TXT===================================================================" << std::endl;
            generateCMakeLists(projectName, getPackagePrefix("custom_msg_visualizer") + "/share/custom_msg_visualizer/base_files/base_cmakelists.txt", outputPackageDir + "/CMakeLists.txt", convertToPackageName(selectedMessage), convertToMessageName(selectedMessage));
            std::cout << "PACKAGE XML===================================================================" << std::endl;
            generatePackageXML(projectName, getPackagePrefix("custom_msg_visualizer") + "/share/custom_msg_visualizer/base_files/base_package.xml", outputPackageDir + "/package.xml", convertToPackageName(selectedMessage));
        }
    }
}