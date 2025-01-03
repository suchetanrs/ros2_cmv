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

#ifndef HPP_CPP_GEENERATOR_HPP_
#define HPP_CPP_GEENERATOR_HPP_

#include "custom_msg_visualizer/cmv_common.hpp"
#include "custom_msg_visualizer/cmv_macros.hpp"
#include INCLUDE_PROJECT_HEADER(custom_msg_metadata.hpp)

namespace custom_msg_visualizer
{
    // Function to parse a .msg file and return a vector of Message structs
    std::vector<Message> parseMsgFile(const std::string &msg_file_path, std::vector<int> &valid_line_idx);

    // Function to generate custom_msg_metadata.hpp file.
    void generateMetadataHeader(std::vector<Message> &messages, const std::string &output_file,
                                const std::string &CUSTOM_MESSAGE_HEADER, const std::string &CUSTOM_MESSAGE_TYPE);

    void generateProcessMsgFile(std::vector<Message> &messages, const std::string &output_file);

    // Function to copy a file from input_path to output_path
    void copyFile(const std::string &input_path, const std::string &output_path);
};

#endif