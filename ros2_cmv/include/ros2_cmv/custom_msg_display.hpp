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

// Helper macro to convert a macro value to a string
#define STRINGIFY_HELPER(x) #x
#define STRINGIFY(x) STRINGIFY_HELPER(x)

// Macro to concatenate PROJECT_NAME with a header file
#define INCLUDE_PROJECT_HEADER(header) STRINGIFY(PROJECT_NAME/header)

#include INCLUDE_PROJECT_HEADER(custom_msg_metadata.hpp)

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
