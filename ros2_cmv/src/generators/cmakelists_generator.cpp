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

#include <ros2_cmv/generators/cmakelists_generator.hpp>

namespace ros2_cmv
{
    void generateCMakeLists(const std::string &project_name, const std::string &input_file,
                            const std::string &output_file, std::string additional_package,
                            const std::string &messageName)
    {
        // Read the content of the input file
        std::ifstream ifs(input_file);
        if (!ifs.is_open())
        {
            std::cerr << "Error: Unable to open " << input_file << " for reading.";
            throw std::runtime_error("Unable to open " + input_file + " for reading.");
        }

        std::ostringstream content_stream;
        content_stream << ifs.rdbuf();
        std::string content = content_stream.str();
        ifs.close();

        // 1. Replace the project name in the content
        size_t pos = content.find("project(");
        if (pos != std::string::npos)
        {
            size_t start = pos + 8; // Skip "project("
            size_t end = content.find(")", start);
            if (end != std::string::npos)
            {
                content.replace(start, end - start, project_name);
            }
        }

        // 2. Add find_package for the additional package as REQUIRED
        std::string find_package_entry = "find_package(" + additional_package + " REQUIRED)";
        size_t find_pkg_pos = content.find("find_package(" + additional_package);
        if (find_pkg_pos == std::string::npos)
        {
            // Insert the find_package entry after the last existing find_package
            size_t last_find_pkg = content.rfind("find_package(");
            if (last_find_pkg != std::string::npos)
            {
                size_t insert_pos = content.find("\n", last_find_pkg);
                if (insert_pos != std::string::npos)
                {
                    content.insert(insert_pos + 1, find_package_entry + "\n");
                    std::cout << "Added find_package entry for '" << additional_package << "'." << std::endl;
                }
                else
                {
                    // If no newline found after last find_package, append at the end
                    content += "\n" + find_package_entry + "\n";
                    std::cout << "Appended find_package entry for '" << additional_package << "' at the end." << std::endl;
                }
            }
            else
            {
                throw std::runtime_error("Could not find any find_package entries in the base CMakeLists.txt.");
            }
        }
        else
        {
            std::cout << "find_package for '" << additional_package << "' already exists." << std::endl;
        }

        // 3. Add the additional package to the dependencies set
        std::string dependencies_start = "set(dependencies";
        size_t dep_pos = content.find(dependencies_start);
        if (dep_pos != std::string::npos)
        {
            size_t open_paren = content.find("(", dep_pos);
            size_t close_paren = content.find(")", open_paren);
            if (open_paren != std::string::npos && close_paren != std::string::npos)
            {
                std::string dependencies_block = content.substr(open_paren + 1, close_paren - open_paren - 1);
                if (dependencies_block.find(additional_package) == std::string::npos)
                {
                    // Insert the additional package before the closing parenthesis
                    content.insert(close_paren, additional_package);
                    std::cout << "Added '" << additional_package << "' to dependencies." << std::endl;
                }
                else
                {
                    std::cout << "Dependency '" << additional_package << "' already exists." << std::endl;
                }
            }
            else
            {
                std::cerr << "Warning: Malformed dependencies set. Unable to add dependency.";
                throw std::runtime_error("Malformed dependencies set in the base CMakeLists.txt.");
            }
        }
        else
        {
            throw std::runtime_error("Could not find dependencies set in the base CMakeLists.txt.");
        }

        // 4. Replace MESSAGE_NAME
        std::string message_name_key = "set(MESSAGE_NAME \"";
        size_t msg_name_pos = content.find(message_name_key);
        if (msg_name_pos != std::string::npos)
        {
            size_t start = msg_name_pos + message_name_key.length();
            size_t end = content.find("\"", start);
            if (end != std::string::npos)
            {
                content.replace(start, end - start, messageName);
                std::cout << "Replaced MESSAGE_NAME with '" << messageName << "'." << std::endl;
            }
            else
            {
                std::cerr << "Malformed MESSAGE_NAME entry in the CMakeLists.txt.";
                throw std::runtime_error("Malformed MESSAGE_NAME entry in the base CMakeLists.txt.");
            }
        }
        else
        {
            std::cout << "No MESSAGE_NAME entry found to replace." << std::endl;
        }

        // Write the modified content to the output file
        std::ofstream ofs(output_file);
        if (!ofs.is_open())
        {
            std::cerr << "Error: Unable to open " << output_file << " for writing.";
            throw std::runtime_error("Unable to open " + output_file + " for writing.");
        }

        ofs << content;
        ofs.close();

        std::cout << "Generated " << output_file << " with project name: " << project_name << std::endl;
    }
}