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

#ifndef AMENT_INDEX_HELPERS_
#define AMENT_INDEX_HELPERS_

#include <ament_index_cpp/get_resources.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include "ros2_cmv/cmv_common.hpp"

namespace ros2_cmv
{
    struct Interface
    {
        std::string package;
        std::string base_name;
    };

    std::vector<Interface> getMsgInterfaces();

    std::string getMsgPath(std::string &package_name, std::string &msg_name);

    std::string getPackagePrefix(std::string package_name);
};

#endif