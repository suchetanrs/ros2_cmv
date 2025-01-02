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

#ifndef CMAKELISTS_GENERATOR_HPP_
#define CMAKELISTS_GENERATOR_HPP_

#include "ros2_cmv/cmv_common.hpp"

namespace ros2_cmv
{
    // Function to generate CMakeLists.txt...Replaces ${PROJECT_NAME} within the CMakeLists.txt
    // project_name: To replace in cmakelists.txt
    // input_file: The input base file to read from
    // output_file: The output file to write to
    // additional_package: The additional package to add to the CMakeLists.txt
    void generateCMakeLists(const std::string &project_name, const std::string &input_file,
                            const std::string &output_file, std::string additional_package,
                            const std::string &messageName);
};

#endif