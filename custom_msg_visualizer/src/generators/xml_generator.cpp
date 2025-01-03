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

#include <custom_msg_visualizer/generators/xml_generator.hpp>

namespace custom_msg_visualizer
{
    void generatePluginXML(const std::string &messageName, const std::string &pluginName, const std::string &output_file,
                           const std::string &project_name)
    {
        try
        {
            // Open the output file
            std::ofstream ofs(output_file);
            if (!ofs.is_open())
            {
                std::cerr << "Error: Unable to open " << output_file << " for writing.";
                throw std::runtime_error("Unable to open " + output_file + " for writing.");
            }

            // Write the XML content
            ofs << "<?xml version=\"1.0\"?>\n";
            ofs << "<library path=\"" << project_name << "_" << messageName << "_rviz_plugin\">\n";
            // ofs << "<library path=\"" << libpath << "\">\n";
            ofs << "  <class name=\"" << pluginName << "\" type=\"" << messageName << "::" << "CustomMessageDisplay" << "\" base_class_type=\"rviz_common::Display\">\n";
            ofs << "    <description>Custom message display for RViz</description>\n";
            ofs << "  </class>\n";
            ofs << "</library>\n";

            // Close the file
            ofs.close();
            std::cout << "Generated " << output_file << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error: " << e.what();
            throw std::runtime_error("Error generating " + output_file + ": " + e.what());
        }
    }

    void generatePackageXML(const std::string &new_name, const std::string &input_file, const std::string &output_file,
                            std::string additional_package)
    {
        try
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

            // 1. Replace the <name>...</name> tag
            size_t start_pos = content.find("<name>");
            size_t end_pos = content.find("</name>", start_pos);
            if (start_pos != std::string::npos && end_pos != std::string::npos)
            {
                start_pos += 6; // Move past the "<name>" tag
                content.replace(start_pos, end_pos - start_pos, new_name);
            }
            else
            {
                std::cerr << "Error: <name> tag not found in " << input_file;
                throw std::runtime_error("Error: <name> tag not found in " + input_file);
            }

            // 2. Replace the <depend>...</depend> tag
            size_t last_depend_end_pos = content.rfind("</depend>");
            if (last_depend_end_pos != std::string::npos)
            {
                // Check if the dependency already exists
                const std::string dependency_tag = "<depend>" + additional_package + "</depend>";
                if (content.find(dependency_tag) == std::string::npos)
                {
                    // Dependency does not exist; insert it
                    last_depend_end_pos += 9; // Move past the last "</depend>"
                    const std::string new_line = "\n  " + dependency_tag;
                    content.insert(last_depend_end_pos, new_line);
                }
                else
                {
                    std::cout << "Dependency '" << additional_package << "' already exists in the package.xml file." << std::endl;
                }
            }
            else
            {
                throw std::runtime_error("Error: No <depend> lines found in " + input_file);
            }

            // Write the modified content to the output file
            std::ofstream ofs(output_file);
            if (!ofs.is_open())
            {
                std::cerr << "Error: Unable to open " << output_file << " for writing.";
                return;
            }

            ofs << content;
            ofs.close();

            std::cout << "Generated " << output_file << " with package name: " << new_name << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error: " << e.what();
            throw std::runtime_error("Error generating " + output_file + ": " + e.what());
        }
    }
}