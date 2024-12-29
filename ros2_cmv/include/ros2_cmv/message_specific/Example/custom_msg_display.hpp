#ifndef CUSTOM_MSG_DISPLAY_HPP_
#define CUSTOM_MSG_DISPLAY_HPP_

#include <rviz_common/display.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include "ros2_cmv/cmv_macros.hpp"
#include INCLUDE_PROJECT_HEADER(custom_msg_process.hpp)

namespace PROJECT_NAME
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

    private:
        std::mutex displayMutex_;
        std::unordered_map<std::string, std::shared_ptr<ros2_cmv::IExposedDisplay>> displayInstances_;
        std::unordered_map<std::string, std::shared_ptr<ros2_cmv::IExposedDisplay>> enabledInstances_;
        std::vector<rviz_common::properties::BoolProperty *> memberVisibilityProperties_;
    };

}; // namespace PROJECT_NAME

#endif // CUSTOM_MSG_DISPLAY_HPP_
