// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish/macros.hpp>

// --- ROS 2 message includes ---
#include <std_msgs/msg/header.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <nav_msgs/msg/grid_cells.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Forward decl
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

// ----------------- Typed casting dispatcher -----------------

using CompoundMessage = ros_babel_fish::CompoundMessage;

static const std::unordered_map<std::string, std::function<void(const CompoundMessage &)>> &handlers()
{
    static const std::unordered_map<std::string, std::function<void(const CompoundMessage &)>> k = {
        // std_msgs
        {"std_msgs/msg/Header", [](const CompoundMessage &m)
         {
             (void)m.message<std_msgs::msg::Header>();
             std::cout << "Cast OK: std_msgs/msg/Header\n";
         }},

        // geometry_msgs points / polygons
        {"geometry_msgs/msg/Point", [](const CompoundMessage &m)
         {
             (void)m.message<geometry_msgs::msg::Point>();
             std::cout << "Cast OK: geometry_msgs/msg/Point\n";
         }},
        {"geometry_msgs/msg/Point32", [](const CompoundMessage &m)
         {
             (void)m.message<geometry_msgs::msg::Point32>();
             std::cout << "Cast OK: geometry_msgs/msg/Point32\n";
         }},
        {"geometry_msgs/msg/PointStamped", [](const CompoundMessage &m)
         {
             (void)m.message<geometry_msgs::msg::PointStamped>();
             std::cout << "Cast OK: geometry_msgs/msg/PointStamped\n";
         }},
        {"geometry_msgs/msg/Polygon", [](const CompoundMessage &m)
         {
             (void)m.message<geometry_msgs::msg::Polygon>();
             std::cout << "Cast OK: geometry_msgs/msg/Polygon\n";
         }},
        {"geometry_msgs/msg/PolygonStamped", [](const CompoundMessage &m)
         {
             (void)m.message<geometry_msgs::msg::PolygonStamped>();
             std::cout << "Cast OK: geometry_msgs/msg/PolygonStamped\n";
         }},

        // geometry_msgs poses
        {"geometry_msgs/msg/Pose", [](const CompoundMessage &m)
         {
             (void)m.message<geometry_msgs::msg::Pose>();
             std::cout << "Cast OK: geometry_msgs/msg/Pose\n";
         }},
        {"geometry_msgs/msg/PoseStamped", [](const CompoundMessage &m)
         {
             (void)m.message<geometry_msgs::msg::PoseStamped>();
             std::cout << "Cast OK: geometry_msgs/msg/PoseStamped\n";
         }},
        {"geometry_msgs/msg/PoseArray", [](const CompoundMessage &m)
         {
             (void)m.message<geometry_msgs::msg::PoseArray>();
             std::cout << "Cast OK: geometry_msgs/msg/PoseArray\n";
         }},
        {"geometry_msgs/msg/PoseWithCovariance", [](const CompoundMessage &m)
         {
             (void)m.message<geometry_msgs::msg::PoseWithCovariance>();
             std::cout << "Cast OK: geometry_msgs/msg/PoseWithCovariance\n";
         }},
        {"geometry_msgs/msg/PoseWithCovarianceStamped", [](const CompoundMessage &m)
         {
             (void)m.message<geometry_msgs::msg::PoseWithCovarianceStamped>();
             std::cout << "Cast OK: geometry_msgs/msg/PoseWithCovarianceStamped\n";
         }},

        // geometry_msgs accel
        {"geometry_msgs/msg/Accel", [](const CompoundMessage &m)
         {
             (void)m.message<geometry_msgs::msg::Accel>();
             std::cout << "Cast OK: geometry_msgs/msg/Accel\n";
         }},
        {"geometry_msgs/msg/AccelStamped", [](const CompoundMessage &m)
         {
             (void)m.message<geometry_msgs::msg::AccelStamped>();
             std::cout << "Cast OK: geometry_msgs/msg/AccelStamped\n";
         }},

        // geometry_msgs twist
        {"geometry_msgs/msg/Twist", [](const CompoundMessage &m)
         {
             (void)m.message<geometry_msgs::msg::Twist>();
             std::cout << "Cast OK: geometry_msgs/msg/Twist\n";
         }},
        {"geometry_msgs/msg/TwistStamped", [](const CompoundMessage &m)
         {
             (void)m.message<geometry_msgs::msg::TwistStamped>();
             std::cout << "Cast OK: geometry_msgs/msg/TwistStamped\n";
         }},

        // geometry_msgs wrench
        {"geometry_msgs/msg/Wrench", [](const CompoundMessage &m)
         {
             (void)m.message<geometry_msgs::msg::Wrench>();
             std::cout << "Cast OK: geometry_msgs/msg/Wrench\n";
         }},
        {"geometry_msgs/msg/WrenchStamped", [](const CompoundMessage &m)
         {
             (void)m.message<geometry_msgs::msg::WrenchStamped>();
             std::cout << "Cast OK: geometry_msgs/msg/WrenchStamped\n";
         }},

        // nav_msgs
        {"nav_msgs/msg/GridCells", [](const CompoundMessage &m)
         {
             (void)m.message<nav_msgs::msg::GridCells>();
             std::cout << "Cast OK: nav_msgs/msg/GridCells\n";
         }},
        {"nav_msgs/msg/Path", [](const CompoundMessage &m)
         {
             (void)m.message<nav_msgs::msg::Path>();
             std::cout << "Cast OK: nav_msgs/msg/Path\n";
         }},
        {"nav_msgs/msg/Odometry", [](const CompoundMessage &m)
         {
             (void)m.message<nav_msgs::msg::Odometry>();
             std::cout << "Cast OK: nav_msgs/msg/Odometry\n";
         }},
        {"nav_msgs/msg/OccupancyGrid", [](const CompoundMessage &m)
         {
             (void)m.message<nav_msgs::msg::OccupancyGrid>();
             std::cout << "Cast OK: nav_msgs/msg/OccupancyGrid\n";
         }},

        // sensor_msgs
        {"sensor_msgs/msg/CameraInfo", [](const CompoundMessage &m)
         {
             (void)m.message<sensor_msgs::msg::CameraInfo>();
             std::cout << "Cast OK: sensor_msgs/msg/CameraInfo\n";
         }},
        {"sensor_msgs/msg/LaserScan", [](const CompoundMessage &m)
         {
             (void)m.message<sensor_msgs::msg::LaserScan>();
             std::cout << "Cast OK: sensor_msgs/msg/LaserScan\n";
         }},
        {"sensor_msgs/msg/PointCloud2", [](const CompoundMessage &m)
         {
             (void)m.message<sensor_msgs::msg::PointCloud2>();
             std::cout << "Cast OK: sensor_msgs/msg/PointCloud2\n";
         }},
        {"sensor_msgs/msg/Range", [](const CompoundMessage &m)
         {
             (void)m.message<sensor_msgs::msg::Range>();
             std::cout << "Cast OK: sensor_msgs/msg/Range\n";
         }},

        // visualization_msgs
        {"visualization_msgs/msg/Marker", [](const CompoundMessage &m)
         {
             (void)m.message<visualization_msgs::msg::Marker>();
             std::cout << "Cast OK: visualization_msgs/msg/Marker\n";
         }},
        {"visualization_msgs/msg/MarkerArray", [](const CompoundMessage &m)
         {
             (void)m.message<visualization_msgs::msg::MarkerArray>();
             std::cout << "Cast OK: visualization_msgs/msg/MarkerArray\n";
         }},
    };
    return k;
}

static void printTypedMessage(const CompoundMessage &msg)
{
    const auto &type = msg.name();
    auto it = handlers().find(type);
    if (it != handlers().end())
    {
        // Try the cast; if it fails, BabelFish will throw and be caught upstream.
        it->second(msg);
    }
    else
    {
        std::cout << "No handler for: " << type << std::endl;
    }
}

// ----------------- Structure walker (minimal) -----------------

void printMessageInformation(const ros_babel_fish::Message &message, const std::string &prefix)
{
    using namespace ros_babel_fish;

    if (message.type() != MessageTypes::Compound)
    {
        std::cout << prefix << "(non-compound message)\n";
        return;
    }

    auto &compound = message.as<CompoundMessage>();
    std::cout << "compound.name = " << compound.name() << std::endl
              << std::endl;

    for (size_t i = 0; i < compound.keys().size(); ++i)
    {
        std::cout << prefix << compound.keys()[i] << " = key " << i << std::endl;
        auto &child_msg = *compound.values()[i];

        if (child_msg.type() == MessageTypes::Compound)
        {
            auto &sub = child_msg.as<CompoundMessage>();
            std::cout << prefix << sub.name() << std::endl;

            // Attempt type-aware cast & "success" print
            try
            {
                printTypedMessage(sub);
            }
            catch (const ros_babel_fish::BabelFishException &e)
            {
                std::cout << "Cast FAILED for: " << sub.name() << " (" << e.what() << ")\n";
            }
        }
        std::cout << std::endl;
    }
}
