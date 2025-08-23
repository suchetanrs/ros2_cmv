// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish/macros.hpp>

/*!
 * The following example demonstrates how to subscribe to a topic of any type,
 *   retrieve the topic's message type, and traverse the message's content.
 */

void printMessageInformation(const ros_babel_fish::Message &message, const std::string &prefix = "");

using std::placeholders::_1;

class AnySubscriber : public rclcpp::Node
{
public:
    explicit AnySubscriber(std::string topic)
        : Node("any_subscriber"), topic_(std::move(topic))
    {
    }

    void init()
    {
        fish = ros_babel_fish::BabelFish::make_shared();
        RCLCPP_INFO(this->get_logger(), "Started. Waiting for topic '%s' to become available.",
                    topic_.c_str());
        subscription_ = fish->create_subscription(
            *this, topic_, 10,
            [this](ros_babel_fish::CompoundMessage::SharedPtr msg)
            { topic_callback(msg); });
        RCLCPP_INFO(this->get_logger(), "Subscribed to topic '%s'", topic_.c_str());
    }

private:
    void topic_callback(const ros_babel_fish::CompoundMessage::SharedPtr &msg) const
    {
        if (msg == nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "I heard a null message");
            return;
        }
        try
        {
            std::cout << "Type: " << msg->name() << std::endl;
            printMessageInformation(*msg);
            std::cout << "---" << std::endl;
        }
        catch (ros_babel_fish::BabelFishException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Got a BabelFishException during translation: %s", ex.what());
        }
        rclcpp::shutdown();
    }

    std::string topic_;
    ros_babel_fish::BabelFish::SharedPtr fish;
    ros_babel_fish::BabelFishSubscription::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cout << "Invalid argument!" << std::endl;
        std::cout << "Usage: any_subscriber [TOPIC]" << std::endl;
        return 1;
    }
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AnySubscriber>(argv[1]);
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// Here's where the dumping happens
void printPoint(const ros_babel_fish::CompoundMessage &point)
{
    // This method demonstrates how to get a known message type out of a CompoundMessage
    // Note that this is only possible if the CompoundMessage actually contains a message of the given type
    // Otherwise, a BabelFishException will be thrown
    // Also, this message is a reference to the original message, so any modifications to either will affect the other
    auto point_msg = point.message<geometry_msgs::msg::Point>();
    std::cout << "Point { x: " << point_msg->x << ", y: " << point_msg->y << ", z: " << point_msg->z
              << " }" << std::endl;
}

void printPoint32(const ros_babel_fish::CompoundMessage &point)
{
    // Again for Point32 so it can be tested with geometry_msgs/msg/Polygon
    auto point_msg = point.message<geometry_msgs::msg::Point32>();
    std::cout << "Point { x: " << point_msg->x << ", y: " << point_msg->y << ", z: " << point_msg->z
              << " }" << std::endl;
}

void printMessageInformation(const ros_babel_fish::Message &message, const std::string &prefix) 
{
    using namespace ros_babel_fish;
    if (message.type() == MessageTypes::Compound)
    {
        auto &compound = message.as<CompoundMessage>();
        std::cout << "compound.name = " << compound.name() <<  std::endl;
        std::cout << std::endl;
        for (size_t i = 0; i < compound.keys().size(); ++i)
        {
            std::cout << prefix << compound.keys()[i] << " = key " << i << std::endl;
            auto& message = *compound.values()[i];
            if (message.type() == MessageTypes::Compound)
            {
                auto& compounded_msg = message.as<CompoundMessage>();
                std::cout << prefix << compounded_msg.name() << std::endl;
            }
            std::cout << std::endl;
        }
    }
}