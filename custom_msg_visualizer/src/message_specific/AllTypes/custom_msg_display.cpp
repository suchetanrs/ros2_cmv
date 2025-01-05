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

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_rendering/material_manager.hpp>

#include "custom_msg_visualizer/cmv_macros.hpp"
#include INCLUDE_PROJECT_HEADER(custom_msg_display.hpp)

namespace MESSAGE_NAME
{
    CustomMessageDisplay::CustomMessageDisplay()
    {
        auto displayFactoryInstance = custom_msg_visualizer::DisplayFactory::instance();
        
        // cleanup the messages that are not supported.
        bool allOk = true;
        for (size_t i = 0; i < variableTypes.size();)
        {
            if (displayFactoryInstance.getCreators().count(variableTypes[i]) == 0)
            {
                std::cout << "Display for : " << variableTypes[i] << " is not created." << std::endl;
                variableTypes.erase(variableTypes.begin() + i);
                variableNames.erase(variableNames.begin() + i);
                allOk = false;
            }
            else
            {
                ++i;
            }
        }
        
        // cleanup array messages that are not supported.
        for (size_t i = 0; i < variableTypesArrays.size();)
        {
            if (displayFactoryInstance.getCreators().count(variableTypesArrays[i]) == 0)
            {
                std::cout << "Display for : " << variableTypesArrays[i] << "[] is not created." << std::endl;
                variableTypesArrays.erase(variableTypesArrays.begin() + i);
                variableNamesArrays.erase(variableNamesArrays.begin() + i);
                allOk = false;
            }
            else
            {
                ++i;
            }
        }
        if (!allOk)
            std::cout << "!!! Use the rolling distro of ROS for the support of these message types. !!! " << std::endl;

        // create the display instances.
        enableAllMembersProperty_ = new rviz_common::properties::BoolProperty(
            "Enable all members",
            false,
            "Whether or not all members should be shown.",
            this,
            SLOT(enableAllMembersChanged()));

        messageMemberProperty_ = new rviz_common::properties::Property("Message Members", QVariant(), "", this);

        for (auto &name : variableNames)
        {
            auto showMemberProperty_ = new rviz_common::properties::BoolProperty(
                QString("Show ") + QString::fromStdString(name),
                false,
                "Whether or not names should be shown next to the frames.",
                messageMemberProperty_,
                SLOT(updateMemberVisibility()),
                this);

            memberVisibilityProperties_.push_back(showMemberProperty_);
        }
    }

    CustomMessageDisplay::~CustomMessageDisplay()
    {
    }

    void CustomMessageDisplay::onEnable()
    {
        {
            std::lock_guard<std::mutex> lock(displayMutex_);
            for (const auto &[key, instance] : displayInstances_)
            {
                instance->onEnable();
            }
        }
        updateMemberVisibility();
    }

    void CustomMessageDisplay::onInitialize()
    {
        std::lock_guard<std::mutex> lock(displayMutex_);
        MFDClass::onInitialize();
        for (size_t i = 0; i < variableTypes.size(); ++i)
        {
            auto instance = custom_msg_visualizer::DisplayFactory::instance().createDisplay(variableTypes[i], context_);
            displayInstances_[variableNames[i]] = instance;
            enabledInstances_[variableNames[i]] = instance;
            instance->onInitialize();
        }
        arrayMessageAssist_ = std::make_shared<ArrayMessageAssist>(context_);
        arrayMessageAssist_->initialize();
    }

    void CustomMessageDisplay::update(float wall_dt, float ros_dt)
    {
        std::lock_guard<std::mutex> lock(displayMutex_);
        for (const auto &[key, instance] : displayInstances_)
        {
            instance->update(wall_dt, ros_dt);
        }
        arrayMessageAssist_->update(wall_dt, ros_dt);
    }

    void CustomMessageDisplay::reset()
    {
        std::lock_guard<std::mutex> lock(displayMutex_);
        MFDClass::reset();
        for (const auto &[key, instance] : displayInstances_)
        {
            instance->reset();
        }
        arrayMessageAssist_->reset();
    }

    void CustomMessageDisplay::processMessage(CustomMessage::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(displayMutex_);
        processCustomMessage(msg, enabledInstances_);
        processCustomMessageVectors(msg, arrayMessageAssist_);
    }

    void CustomMessageDisplay::updateMemberVisibility()
    {
        std::lock_guard<std::mutex> lock(displayMutex_);
        for (size_t idx = 0; idx < memberVisibilityProperties_.size(); idx++)
        {
            auto property = memberVisibilityProperties_[idx];
            if (property->getBool() == false)
            {
                displayInstances_[variableNames[idx]]->reset();
                enabledInstances_.erase(variableNames[idx]);
            }
            else if (property->getBool() == true)
            {
                enabledInstances_[variableNames[idx]] = displayInstances_[variableNames[idx]];
            }
        }
    }

    void CustomMessageDisplay::enableAllMembersChanged()
    {
        bool enabledAllMembers = enableAllMembersProperty_->getBool();
        for (size_t idx = 0; idx < memberVisibilityProperties_.size(); idx++)
        {
            auto property = memberVisibilityProperties_[idx];
            property->setBool(enabledAllMembers);
        }
        updateMemberVisibility();
    }

} // namespace MESSAGE_NAME

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MESSAGE_NAME::CustomMessageDisplay, rviz_common::Display)