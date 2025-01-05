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

#include "custom_msg_visualizer/cmv_macros.hpp"
#include INCLUDE_PROJECT_HEADER(custom_msg_metadata.hpp)

#include "custom_msg_visualizer/exposed_displays.hpp"
#include "custom_msg_visualizer/cmv_macros.hpp"
#include INCLUDE_PROJECT_HEADER(array_assist.hpp)

namespace MESSAGE_NAME
{
    void processCustomMessage(const CustomMessage::ConstSharedPtr &msg, std::unordered_map<std::string, std::shared_ptr<custom_msg_visualizer::IExposedDisplay>> &enabledInstances);

    void processCustomMessageVectors(const CustomMessage::ConstSharedPtr &msg, std::shared_ptr<ArrayMessageAssist> arrayAssist);
}

#endif