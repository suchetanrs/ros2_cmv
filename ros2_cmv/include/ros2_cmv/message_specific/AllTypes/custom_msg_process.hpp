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

#ifndef CUSTOM_MSG_PROCESS_HPP_
#define CUSTOM_MSG_PROCESS_HPP_

#include "ros2_cmv/cmv_macros.hpp"
#include INCLUDE_PROJECT_HEADER(custom_msg_metadata.hpp)

#include "ros2_cmv/exposed_displays.hpp"

namespace ros2_cmv
{
    void processCustomMessage(const CustomMessage::ConstSharedPtr &msg, std::unordered_map<std::string, std::shared_ptr<IExposedDisplay>> &enabledInstances);
}

#endif