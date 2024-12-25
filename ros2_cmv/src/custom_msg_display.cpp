#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_rendering/material_manager.hpp>

// Helper macro to convert a macro value to a string
#define STRINGIFY_HELPER(x) #x
#define STRINGIFY(x) STRINGIFY_HELPER(x)

// Macro to concatenate PROJECT_NAME with a header file
#define INCLUDE_PROJECT_HEADER(header) STRINGIFY(PROJECT_NAME/header)

#include INCLUDE_PROJECT_HEADER(custom_msg_display.hpp)

namespace PROJECT_NAME
{
    CustomMessageDisplay::CustomMessageDisplay()
    {
        for (auto &name : variableNames)
        {
            auto showMemberProperty_ = new rviz_common::properties::BoolProperty(
                QString("Show ") + QString::fromStdString(name),
                false,
                "Whether or not names should be shown next to the frames.",
                this,
                SLOT(updateMemberVisibility()));

            memberVisibilityProperties_.push_back(showMemberProperty_);
        }
    }

    CustomMessageDisplay::~CustomMessageDisplay()
    {
        // Clean up if necessary
    }

    void CustomMessageDisplay::onEnable()
    {
        for (const auto &[key, instance] : displayInstances_)
        {
            instance->onEnable();
        }
    }

    void CustomMessageDisplay::onInitialize()
    {
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
        for (const auto &[key, instance] : displayInstances_)
        {
            instance->update(wall_dt, ros_dt);
        }
    }

    void CustomMessageDisplay::reset()
    {
        MFDClass::reset();
        for (const auto &[key, instance] : displayInstances_)
        {
            instance->reset();
        }
    }

    void CustomMessageDisplay::processMessage(CustomMessage::ConstSharedPtr msg)
    {
        processCustomMessage(msg, enabledInstances_);
    }

    void CustomMessageDisplay::updateMemberVisibility()
    {
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

} // namespace ros2_cmv

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PROJECT_NAME::CustomMessageDisplay, rviz_common::Display)