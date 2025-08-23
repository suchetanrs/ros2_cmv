#include <rclcpp/rclcpp.hpp>

#include <ros_babel_fish/idl/providers/local_type_support_provider.hpp>
#include <ros_babel_fish/idl/type_support.hpp>
#include <ros_babel_fish/messages/compound_message.hpp>
#include <ros_babel_fish/messages/message.hpp>
#include <ros_babel_fish/messages/message_types.hpp>

#include <iostream>
#include <string>
#include <unordered_map>

using ros_babel_fish::CompoundMessage;
using ros_babel_fish::LocalTypeSupportProvider;
using ros_babel_fish::Message;
using ros_babel_fish::MessageMembersIntrospection;
using ros_babel_fish::MessageTypeSupport;

static std::string message_type_to_string(ros_babel_fish::MessageType t)
{
    using ros_babel_fish::MessageType;
    switch (t)
    {
    case MessageType::Bool:
        return "bool";
    case MessageType::Octet:
        return "octet";
    case MessageType::UInt8:
        return "uint8";
    case MessageType::UInt16:
        return "uint16";
    case MessageType::UInt32:
        return "uint32";
    case MessageType::UInt64:
        return "uint64";
    case MessageType::Int8:
        return "int8";
    case MessageType::Int16:
        return "int16";
    case MessageType::Int32:
        return "int32";
    case MessageType::Int64:
        return "int64";
    case MessageType::Float:
        return "float";
    case MessageType::Double:
        return "double";
    case MessageType::LongDouble:
        return "long double";
    case MessageType::Char:
        return "char";
    case MessageType::WChar:
        return "wchar";
    case MessageType::String:
        return "string";
    case MessageType::WString:
        return "wstring";
    case MessageType::Compound:
        return "compound";
    case MessageType::Array:
        return "array";
    case MessageType::None:
    default:
        return "unknown";
    }
}

class IntrospectNode : public rclcpp::Node
{
public:
    IntrospectNode() : Node("rbf_introspect")
    {
        // Fully-qualified type name like "geometry_msgs/msg/Pose".
        msg_type_ = this->declare_parameter<std::string>(
            "message_type", "custom_msg_visualizer_example/msg/CorridorData");

        try
        {
            LocalTypeSupportProvider provider;
            auto ts = provider.getMessageTypeSupport(msg_type_);
            if (!ts)
            {
                RCLCPP_ERROR(get_logger(), "Failed to resolve type support for '%s'", msg_type_.c_str());
                rclcpp::shutdown();
                return;
            }

            MessageMembersIntrospection introspection(*ts);
            CompoundMessage compound(introspection);

            std::cout << "Message: " << msg_type_ << "\n";
            std::cout << "Fields (name : type)" << "\n";
            auto names = compound.keys();
            auto vals = compound.values();
            for (size_t i = 0; i < names.size() && i < vals.size(); ++i)
            {
                const std::string &name = names[i];
                const Message &m = *vals[i];
                std::cout << "  " << name << " : " << message_type_to_string(m.type()) << "\n";
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(get_logger(), "Exception: %s", e.what());
        }
    }

private:
    std::string msg_type_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IntrospectNode>();
    rclcpp::spin(node);
    return 0;
}
