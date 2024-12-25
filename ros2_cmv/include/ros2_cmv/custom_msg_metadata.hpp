#ifndef CUSTOM_MSG_TEMPLATES_
#define CUSTOM_MSG_TEMPLATES_

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

#include "ros2_cmv_example/msg/example.hpp"
#include "ros2_cmv/exposed_displays.hpp"

namespace ros2_cmv
{
    // This is just an example having all the supported message types.
    // In the actual generated code, this file will be replaced with the specified message and the message fields.

    using CustomMessage = ros2_cmv_example::msg::Example;

    static const std::vector<std::string> variableTypes = {
        "geometry_msgs/AccelStamped",
        "geometry_msgs/Accel",
        "sensor_msgs/CameraInfo",
        "nav_msgs/GridCells",
        "geometry_msgs/PointStamped",
        "geometry_msgs/PointStamped",
        "geometry_msgs/Point",
        "geometry_msgs/PolygonStamped",
        "geometry_msgs/Polygon",
        "geometry_msgs/PoseStamped",
        "geometry_msgs/Pose",
        "geometry_msgs/PoseArray",
        "geometry_msgs/PoseWithCovarianceStamped",
        "geometry_msgs/PoseWithCovariance",
        "nav_msgs/Path",
        "nav_msgs/Odometry",
        "nav_msgs/OccupancyGrid",
        "visualization_msgs/Marker",
        "visualization_msgs/MarkerArray",
        "sensor_msgs/LaserScan",
        "sensor_msgs/PointCloud2",
        "sensor_msgs/Range",
        "geometry_msgs/TwistStamped",
        "geometry_msgs/Twist",
        "geometry_msgs/WrenchStamped",
        "geometry_msgs/Wrench",
    };

    static const std::vector<std::string> variableNames = {
        "accel",
        "accel_raw",
        "camera_info",
        "grid_cells",
        "point1",
        "point2",
        "point_raw",
        "polygon1",
        "polygon_raw",
        "pose",
        "pose_raw",
        "pose_array",
        "pose_with_covariance",
        "pose_with_covariance_raw",
        "path",
        "odometry",
        "map",
        "marker",
        "marker_array",
        "laser_scan",
        "point_cloud2",
        "range",
        "twist",
        "twist_raw",
        "wrench",
        "wrench_raw",
    };

    inline void processCustomMessage(const CustomMessage::ConstSharedPtr &msg, std::unordered_map<std::string, std::shared_ptr<IExposedDisplay>> &enabledInstances)
    {
        if (enabledInstances.find("accel") != enabledInstances.end())
        {
            enabledInstances["accel"]->processMessage(std::make_shared<const geometry_msgs::msg::AccelStamped>(msg->accel));
        }
        if (enabledInstances.find("accel_raw") != enabledInstances.end())
        {
            enabledInstances["accel_raw"]->processMessage(std::make_shared<const geometry_msgs::msg::Accel>(msg->accel_raw));
        }
        if (enabledInstances.find("camera_info") != enabledInstances.end())
        {
            enabledInstances["camera_info"]->processMessage(std::make_shared<const sensor_msgs::msg::CameraInfo>(msg->camera_info));
        }
        if (enabledInstances.find("grid_cells") != enabledInstances.end())
        {
            enabledInstances["grid_cells"]->processMessage(std::make_shared<const nav_msgs::msg::GridCells>(msg->grid_cells));
        }
        if (enabledInstances.find("point1") != enabledInstances.end())
        {
            enabledInstances["point1"]->processMessage(std::make_shared<const geometry_msgs::msg::PointStamped>(msg->point1));
        }
        if (enabledInstances.find("point2") != enabledInstances.end())
        {
            enabledInstances["point2"]->processMessage(std::make_shared<const geometry_msgs::msg::PointStamped>(msg->point2));
        }
        if (enabledInstances.find("point_raw") != enabledInstances.end())
        {
            enabledInstances["point_raw"]->processMessage(std::make_shared<const geometry_msgs::msg::Point>(msg->point_raw));
        }
        if (enabledInstances.find("polygon1") != enabledInstances.end())
        {
            enabledInstances["polygon1"]->processMessage(std::make_shared<const geometry_msgs::msg::PolygonStamped>(msg->polygon1));
        }
        if (enabledInstances.find("polygon_raw") != enabledInstances.end())
        {
            enabledInstances["polygon_raw"]->processMessage(std::make_shared<const geometry_msgs::msg::Polygon>(msg->polygon_raw));
        }
        if (enabledInstances.find("pose") != enabledInstances.end())
        {
            enabledInstances["pose"]->processMessage(std::make_shared<const geometry_msgs::msg::PoseStamped>(msg->pose));
        }
        if (enabledInstances.find("pose_raw") != enabledInstances.end())
        {
            enabledInstances["pose_raw"]->processMessage(std::make_shared<const geometry_msgs::msg::Pose>(msg->pose_raw));
        }
        if (enabledInstances.find("pose_array") != enabledInstances.end())
        {
            enabledInstances["pose_array"]->processMessage(std::make_shared<const geometry_msgs::msg::PoseArray>(msg->pose_array));
        }
        if (enabledInstances.find("pose_with_covariance") != enabledInstances.end())
        {
            enabledInstances["pose_with_covariance"]->processMessage(std::make_shared<const geometry_msgs::msg::PoseWithCovarianceStamped>(msg->pose_with_covariance));
        }
        if (enabledInstances.find("pose_with_covariance_raw") != enabledInstances.end())
        {
            enabledInstances["pose_with_covariance_raw"]->processMessage(std::make_shared<const geometry_msgs::msg::PoseWithCovariance>(msg->pose_with_covariance_raw));
        }
        if (enabledInstances.find("path") != enabledInstances.end())
        {
            enabledInstances["path"]->processMessage(std::make_shared<const nav_msgs::msg::Path>(msg->path));
        }
        if (enabledInstances.find("odometry") != enabledInstances.end())
        {
            enabledInstances["odometry"]->processMessage(std::make_shared<const nav_msgs::msg::Odometry>(msg->odometry));
        }
        if (enabledInstances.find("map") != enabledInstances.end())
        {
            enabledInstances["map"]->processMessage(std::make_shared<const nav_msgs::msg::OccupancyGrid>(msg->map));
        }
        if (enabledInstances.find("marker") != enabledInstances.end())
        {
            enabledInstances["marker"]->processMessage(std::make_shared<const visualization_msgs::msg::Marker>(msg->marker));
        }
        if (enabledInstances.find("marker_array") != enabledInstances.end())
        {
            enabledInstances["marker_array"]->processMessage(std::make_shared<const visualization_msgs::msg::MarkerArray>(msg->marker_array));
        }
        if (enabledInstances.find("laser_scan") != enabledInstances.end())
        {
            enabledInstances["laser_scan"]->processMessage(std::make_shared<const sensor_msgs::msg::LaserScan>(msg->laser_scan));
        }
        if (enabledInstances.find("point_cloud2") != enabledInstances.end())
        {
            enabledInstances["point_cloud2"]->processMessage(std::make_shared<const sensor_msgs::msg::PointCloud2>(msg->point_cloud2));
        }
        if (enabledInstances.find("range") != enabledInstances.end())
        {
            enabledInstances["range"]->processMessage(std::make_shared<const sensor_msgs::msg::Range>(msg->range));
        }
        if (enabledInstances.find("twist") != enabledInstances.end())
        {
            enabledInstances["twist"]->processMessage(std::make_shared<const geometry_msgs::msg::TwistStamped>(msg->twist));
        }
        if (enabledInstances.find("twist_raw") != enabledInstances.end())
        {
            enabledInstances["twist_raw"]->processMessage(std::make_shared<const geometry_msgs::msg::Twist>(msg->twist_raw));
        }
        if (enabledInstances.find("wrench") != enabledInstances.end())
        {
            enabledInstances["wrench"]->processMessage(std::make_shared<const geometry_msgs::msg::WrenchStamped>(msg->wrench));
        }
        if (enabledInstances.find("wrench_raw") != enabledInstances.end())
        {
            enabledInstances["wrench_raw"]->processMessage(std::make_shared<const geometry_msgs::msg::Wrench>(msg->wrench_raw));
        }
    }
};
#endif // CUSTOM_MSG_TEMPLATES_
