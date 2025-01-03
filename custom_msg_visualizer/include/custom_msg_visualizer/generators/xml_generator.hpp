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

#ifndef PACKAGE_XML_GENERATOR_HPP_
#define PACKAGE_XML_GENERATOR_HPP_

#include "custom_msg_visualizer/cmv_common.hpp"
#include "custom_msg_visualizer/cmv_macros.hpp"

namespace custom_msg_visualizer
{
    // Function to generate rviz_plugin.xml
    void generatePluginXML(const std::string &messageName, const std::string &pluginName,
                           const std::string &output_file, const std::string &project_name);

    // Function to generate package.xml
    void generatePackageXML(const std::string &new_name, const std::string &input_file,
                            const std::string &output_file, std::string additional_package);
};
#endif