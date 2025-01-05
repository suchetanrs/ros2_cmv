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

#ifndef GENERATOR_HPP_
#define GENERATOR_HPP_

#include "custom_msg_visualizer/generators/hpp_cpp_generator.hpp"
#include "custom_msg_visualizer/generators/xml_generator.hpp"
#include "custom_msg_visualizer/generators/cmakelists_generator.hpp"

#include "custom_msg_visualizer/ament_index_helpers.hpp"

namespace custom_msg_visualizer
{
    // Function to check if a file is a valid ROS message type for generation
    bool checkMessageValidity(const std::string &msg_file_path, std::vector<int> &valid_line_idx);

    void separateMessages(const std::vector<Message> &allMessages,
                          std::vector<Message> &messages,
                          std::vector<Message> &arrayMessages);

    void generateFiles(std::string &msgFilePath, std::string &outputPackageDir,
                       std::string &selectedMessage, std::string &projectName,
                       bool newPackage);
};

#endif