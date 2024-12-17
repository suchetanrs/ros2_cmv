#include <ros2_cmv/custom_msg_display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_rendering/material_manager.hpp>

namespace ros2_cmv
{

    CustomMessageDisplay::CustomMessageDisplay()
    {
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
            auto instance = DisplayFactory::instance().createDisplay(variableTypes[i], context_);
            displayInstances_[variableNames[i]] = instance;
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
        processCustomMessage(msg, displayInstances_);
    }

} // namespace ros2_cmv

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ros2_cmv::CustomMessageDisplay, rviz_common::Display)
