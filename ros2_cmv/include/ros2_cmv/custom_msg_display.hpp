#ifndef CUSTOM_MSG_DISPLAY_HPP_
#define CUSTOM_MSG_DISPLAY_HPP_

#include <rviz_common/display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include "ros2_cmv/custom_msg_metadata.hpp"

namespace ros2_cmv
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

        std::unordered_map<std::string, std::shared_ptr<IExposedDisplay>> displayInstances_;
    };

} // namespace ros2_cmv

#endif // CUSTOM_MSG_DISPLAY_HPP_
