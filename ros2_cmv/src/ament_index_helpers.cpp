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

#include "ros2_cmv/ament_index_helpers.hpp"

namespace ros2_cmv
{
    std::vector<Interface> getMsgInterfaces()
    {
        // Retrieve all packages that have a "rosidl_interfaces" resource
        auto packages = ament_index_cpp::get_resources("rosidl_interfaces");

        std::vector<Interface> known_interfaces;

        for (const auto &package : packages)
        {
            const std::string &package_name = package.first;

            // Get the resource which lists the interfaces in this package
            std::string content;
            if (!ament_index_cpp::get_resource("rosidl_interfaces", package_name, content))
            {
                continue;
            }

            // The content typically lists .msg, .srv, and .action files, one per line.
            // We'll split by newline and extract only .msg files.
            std::istringstream iss(content);
            std::string line;
            while (std::getline(iss, line))
            {
                // Trim whitespace
                if (line.empty())
                {
                    continue;
                }

                // Check if this line corresponds to a message file
                // Lines often look like: "msg/Foo.msg"
                // We can look for a substring ".msg" to identify message interfaces.
                if (line.find(".msg") != std::string::npos)
                {
                    // Format the full message type as "package_name/msg_name"
                    // The line usually looks like "msg/Foo.msg", so we can remove "msg/" and ".msg"
                    std::string base_name = line;
                    if (base_name.find("msg/") == 0)
                    {
                        base_name.erase(0, 4); // remove "msg/"
                    }
                    if (base_name.size() > 4 && base_name.substr(base_name.size() - 4) == ".msg")
                    {
                        base_name.erase(base_name.size() - 4);
                    }
                    Interface known_interface;
                    known_interface.package = package_name;
                    known_interface.base_name = base_name;
                    known_interfaces.push_back(known_interface);
                }
            }
        }
        return known_interfaces;
    };

    std::string getMsgPath(std::string &package_name, std::string &msg_name)
    {
        // Find the package prefix directory
        std::string package_prefix;
        try
        {
            package_prefix = ament_index_cpp::get_package_prefix(package_name);
        }
        catch (const std::runtime_error &e)
        {
            throw std::runtime_error("Could not find package prefix for " + package_name);
        }

        // Construct the path to the .msg file
        // By convention, message files are located in: share/<package_name>/msg/<MsgName>.msg
        std::string msg_file_path = package_prefix + "/share/" + package_name + "/msg/" + msg_name + ".msg";

        return msg_file_path;
    };

    std::string getPackagePrefix(std::string package_name)
    {
        // Find the package prefix directory
        std::string package_prefix;
        try
        {
            package_prefix = ament_index_cpp::get_package_prefix(package_name);
        }
        catch (const std::runtime_error &e)
        {
            throw std::runtime_error("Could not find package prefix for " + package_name + ". Is it installed?");
        }
        return package_prefix;
    };
};