#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_rendering/material_manager.hpp>

#include "ros2_cmv/cmv_macros.hpp"
#include INCLUDE_PROJECT_HEADER(custom_msg_display.hpp)

namespace MESSAGE_NAME
{
    CustomMessageDisplay::CustomMessageDisplay()
    {
        // cleanup the messages that are not supported.
        auto displayFactoryInstance = ros2_cmv::DisplayFactory::instance();
        bool allOk = true;
        for (size_t i = 0; i < variableTypes.size();)
        {
            if (displayFactoryInstance.getCreators().count(variableTypes[i]) == 0)
            {
                variableTypes.erase(variableTypes.begin() + i);
                variableNames.erase(variableNames.begin() + i);
                std::cout << "Display for : " << variableTypes[i] << " is not created." << std::endl;
                allOk = false;
            }
            else
            {
                ++i;
            }
        }
        if (!allOk)
            std::cout << "!!! Use the latest distro of ROS for the support of these message types. !!! " << std::endl;

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
            auto instance = ros2_cmv::DisplayFactory::instance().createDisplay(variableTypes[i], context_);
            displayInstances_[variableNames[i]] = instance;
            enabledInstances_[variableNames[i]] = instance;
            instance->onInitialize();
        }
    }

    void CustomMessageDisplay::update(float wall_dt, float ros_dt)
    {
        std::lock_guard<std::mutex> lock(displayMutex_);
        for (const auto &[key, instance] : displayInstances_)
        {
            instance->update(wall_dt, ros_dt);
        }
    }

    void CustomMessageDisplay::reset()
    {
        std::lock_guard<std::mutex> lock(displayMutex_);
        MFDClass::reset();
        for (const auto &[key, instance] : displayInstances_)
        {
            instance->reset();
        }
    }

    void CustomMessageDisplay::processMessage(CustomMessage::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(displayMutex_);
        processCustomMessage(msg, enabledInstances_);
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