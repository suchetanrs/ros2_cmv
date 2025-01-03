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

#ifndef CUSTOM_MSG_DISPLAY_HPP_
#define CUSTOM_MSG_DISPLAY_HPP_

#include <rviz_common/display.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include "custom_msg_visualizer/cmv_macros.hpp"
#include INCLUDE_PROJECT_HEADER(custom_msg_process.hpp)

namespace MESSAGE_NAME
{
    class CustomMessageDisplay : public rviz_common::MessageFilterDisplay<CustomMessage>
    {
        Q_OBJECT

    public:
        CustomMessageDisplay();
        virtual ~CustomMessageDisplay();
        void onEnable() override;
        void onInitialize() override;
        void update(float wall_dt, float ros_dt) override;

    protected:
        void reset() override;
        void processMessage(CustomMessage::ConstSharedPtr msg) override;
    private Q_SLOTS:
        void updateMemberVisibility();
        void enableAllMembersChanged();

    private:
        std::mutex displayMutex_;
        std::unordered_map<std::string, std::shared_ptr<custom_msg_visualizer::IExposedDisplay>> displayInstances_;
        std::unordered_map<std::string, std::shared_ptr<custom_msg_visualizer::IExposedDisplay>> enabledInstances_;
        std::vector<rviz_common::properties::BoolProperty *> memberVisibilityProperties_;
        rviz_common::properties::BoolProperty *enableAllMembersProperty_;
        rviz_common::properties::Property *messageMemberProperty_;
    };

}; // namespace MESSAGE_NAME

#endif // CUSTOM_MSG_DISPLAY_HPP_
