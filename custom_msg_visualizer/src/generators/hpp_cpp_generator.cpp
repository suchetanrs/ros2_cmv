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

#include <custom_msg_visualizer/generators/hpp_cpp_generator.hpp>

namespace custom_msg_visualizer
{
    std::vector<Message> parseMsgFile(const std::string &msg_file_path, std::vector<int> &valid_line_idx)
    {
        std::vector<Message> messages;
        std::ifstream file(msg_file_path);
        if (!file.is_open())
        {
            std::cerr << "Error: Unable to open " << msg_file_path;
            throw std::runtime_error("Error: Unable to open " + msg_file_path);
        }

        int currIdx = -1;
        std::string line;
        while (std::getline(file, line))
        {
            currIdx++;
            size_t comment_pos = line.find('#');
            if (comment_pos != std::string::npos)
            {
                line = line.substr(0, comment_pos);
            }
            // Trim whitespace
            line = trim(line);
            if (line.empty())
            {
                continue;
            }

            // Split line into parts
            std::vector<std::string> parts = split(line);
            if (parts.size() == 3 || parts.size() == 2)
            {
                std::string msg_type = parts[0];
                std::string var_name = parts[1];
                // std::string default_value = parts[2]; // Not used in C++ generation
                // should be in valid_line_idx
                if (std::find(valid_line_idx.begin(), valid_line_idx.end(), currIdx) != valid_line_idx.end())
                {
                    messages.push_back(Message{msg_type, var_name});
                }
            }
            else
            {
                // std::cerr << "Skipping invalid line: " << line);
            }
        }

        file.close();
        return messages;
    }

    void generateMetadataHeader(std::vector<Message> &messages, std::vector<Message> &arrayMessages, const std::string &output_file,
                                const std::string &CUSTOM_MESSAGE_HEADER, const std::string &CUSTOM_MESSAGE_TYPE)
    {
        std::ofstream ofs(output_file);
        if (!ofs.is_open())
        {
            std::cerr << "Error: Unable to open " << output_file << " for writing.";
            throw std::runtime_error("Error: Unable to open " + output_file + " for writing.");
        }

        // Header Guard
        ofs << "// Generated with ros2_cmv. File has no license.\n\n";
        ofs << "#ifndef CUSTOM_MSG_METADATA_HPP_\n";
        ofs << "#define CUSTOM_MSG_METADATA_HPP_\n\n";

        // Includes
        ofs << "#include <string>\n";
        ofs << "#include <vector>\n";

        ofs << "#include \"" << CUSTOM_MESSAGE_HEADER << "\"\n\n";

        // Using declaration
        ofs << "using CustomMessage = " << CUSTOM_MESSAGE_TYPE << ";\n\n";

        // variableTypes Vector
        ofs << "static std::vector<std::string> variableTypes = {\n";
        for (const auto &msg : messages)
        {
            if (msg.type == "std_msgs/Header")
            {
                continue;
            }
            ofs << "    \"" << msg.type << "\",\n";
        }
        ofs << "};\n\n";

        ofs << "static std::vector<std::string> variableTypesArrays = {\n";
        for (const auto &msg : arrayMessages)
        {
            if (msg.type == "std_msgs/Header")
            {
                continue;
            }
            ofs << "    \"" << msg.type << "\",\n";
        }
        ofs << "};\n\n";

        // variableNames Vector
        ofs << "static std::vector<std::string> variableNames = {\n";
        for (const auto &msg : messages)
        {
            if (msg.type == "std_msgs/Header")
            {
                continue;
            }
            ofs << "    \"" << msg.name << "\",\n";
        }
        ofs << "};\n\n";

        // variableNamesArrays Vector
        ofs << "static std::vector<std::string> variableNamesArrays = {\n";
        for (const auto &msg : arrayMessages)
        {
            if (msg.type == "std_msgs/Header")
            {
                continue;
            }
            ofs << "    \"" << msg.name << "\",\n";
        }
        ofs << "};\n\n";

        // End Header Guard
        ofs << "#endif // CUSTOM_MSG_METADATA_HPP_\n";

        ofs.close();
        std::cout << "Generated " << output_file << std::endl;
    }

    void generateProcessMsgFile(std::vector<Message> &messages, std::vector<Message> &arrayMessages, const std::string &output_file)
    {
        std::ofstream ofs(output_file);
        if (!ofs.is_open())
        {
            std::cerr << "Error: Unable to open " << output_file << " for writing.";
            throw std::runtime_error("Error: Unable to open " + output_file + " for writing.");
        }

        // Includes
        ofs << "// Generated with ros2_cmv. File has no license.\n\n";
        ofs << "#include \"custom_msg_visualizer/cmv_macros.hpp\"\n";
        ofs << "#include INCLUDE_PROJECT_HEADER(custom_msg_process.hpp)\n\n";
        ofs << "namespace MESSAGE_NAME\n{\n";

        // processCustomMessage Function
        ofs << "    void processCustomMessage(const CustomMessage::ConstSharedPtr &msg, std::unordered_map<std::string, std::shared_ptr<custom_msg_visualizer::IExposedDisplay>>& enabledInstances)\n";
        ofs << "    {\n";
        for (const auto &msg : messages)
        {
            if (msg.type == "std_msgs/Header")
            {
                std::string cpp_type = convertRosTypeToCpp(msg.type);
                std::string var_name = msg.name;
                ofs << "        custom_msg_visualizer::globalValues.setHeader(msg->" << var_name << ");\n";
                continue;
            }
            std::string cpp_type = convertRosTypeToCpp(msg.type);
            std::string var_name = msg.name;
            ofs << "        if(enabledInstances.find(\"" << var_name << "\") != enabledInstances.end()) {\n";
            ofs << "            enabledInstances[\"" << var_name << "\"]->processMessage(std::make_shared<const " << cpp_type << ">(msg->" << var_name << "));\n";
            ofs << "        }\n";
        }
        ofs << "    }\n\n";

        // processCustomMessageVectors Function
        ofs << "    void processCustomMessageVectors(const CustomMessage::ConstSharedPtr &msg, std::shared_ptr<ArrayMessageAssist> arrayAssist)\n";
        ofs << "    {\n";
        for (const auto &msg : arrayMessages)
        {
            if (msg.type == "std_msgs/Header")
            {
                continue;
            }
            std::string cpp_type = convertRosTypeToCpp(msg.type);
            std::string var_name = msg.name;
            ofs << "        arrayAssist->processArray<" << cpp_type << ">(msg->" << var_name << ", \"" << var_name << "\", \"" << msg.type << "\");\n";
            // ofs << "        if(enabledInstances.find(\"" << var_name << "\") != enabledInstances.end()) {\n";
            // ofs << "            enabledInstances[\"" << var_name << "\"]->processMessage(std::make_shared<const " << cpp_type << ">(msg->" << var_name << "));\n";
        }
        ofs << "    }\n};\n";

        ofs.close();
        std::cout << "Generated " << output_file << std::endl;
    }

    void copyFile(const std::string &input_path, const std::string &output_path)
    {
        // Open the input file for reading
        std::ifstream input_file(input_path, std::ios::binary);
        if (!input_file.is_open())
        {
            std::cerr << "Error: Unable to open " << input_path << " for reading.";
            input_file.close();
            throw std::runtime_error("Error: Unable to open " + input_path + " for reading.");
        }

        // Open the output file for writing
        std::ofstream output_file(output_path, std::ios::binary);
        if (!output_file.is_open())
        {
            std::cerr << "Error: Unable to open " << output_path << " for writing.";
            output_file.close();
            throw std::runtime_error("Error: Unable to open " + output_path + " for writing.");
        }

        std::string line;
        for (int i = 0; i < 21; ++i)
        {
            if (!std::getline(input_file, line))
            {
                std::cerr << "Warning: Input file has fewer than 20 lines. Nothing to copy.\n";
                return; // Exit the function if there are fewer than 20 lines
            }
        }

        output_file << "// Generated with ros2_cmv. File has no license.\n";

        // Copy the file content
        output_file << input_file.rdbuf();

        // Close the files
        input_file.close();
        output_file.close();

        std::cout << "File copied from " << input_path << " to " << output_path << std::endl;
    }
}