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

    inline void processCustomMessage(const CustomMessage::ConstSharedPtr &msg, std::unordered_map<std::string, std::shared_ptr<IExposedDisplay>> &displayInstances)
    {
        if (displayInstances.find("accel") != displayInstances.end())
        {
            displayInstances["accel"]->processMessage(std::make_shared<const geometry_msgs::msg::AccelStamped>(msg->accel));
        }
        if (displayInstances.find("accel_raw") != displayInstances.end())
        {
            displayInstances["accel_raw"]->processMessage(std::make_shared<const geometry_msgs::msg::Accel>(msg->accel_raw));
        }
        if (displayInstances.find("camera_info") != displayInstances.end())
        {
            displayInstances["camera_info"]->processMessage(std::make_shared<const sensor_msgs::msg::CameraInfo>(msg->camera_info));
        }
        if (displayInstances.find("grid_cells") != displayInstances.end())
        {
            displayInstances["grid_cells"]->processMessage(std::make_shared<const nav_msgs::msg::GridCells>(msg->grid_cells));
        }
        if (displayInstances.find("point1") != displayInstances.end())
        {
            displayInstances["point1"]->processMessage(std::make_shared<const geometry_msgs::msg::PointStamped>(msg->point1));
        }
        if (displayInstances.find("point2") != displayInstances.end())
        {
            displayInstances["point2"]->processMessage(std::make_shared<const geometry_msgs::msg::PointStamped>(msg->point2));
        }
        if (displayInstances.find("point_raw") != displayInstances.end())
        {
            displayInstances["point_raw"]->processMessage(std::make_shared<const geometry_msgs::msg::Point>(msg->point_raw));
        }
        if (displayInstances.find("polygon1") != displayInstances.end())
        {
            displayInstances["polygon1"]->processMessage(std::make_shared<const geometry_msgs::msg::PolygonStamped>(msg->polygon1));
        }
        if (displayInstances.find("polygon_raw") != displayInstances.end())
        {
            displayInstances["polygon_raw"]->processMessage(std::make_shared<const geometry_msgs::msg::Polygon>(msg->polygon_raw));
        }
        if (displayInstances.find("pose") != displayInstances.end())
        {
            displayInstances["pose"]->processMessage(std::make_shared<const geometry_msgs::msg::PoseStamped>(msg->pose));
        }
        if (displayInstances.find("pose_raw") != displayInstances.end())
        {
            displayInstances["pose_raw"]->processMessage(std::make_shared<const geometry_msgs::msg::Pose>(msg->pose_raw));
        }
        if (displayInstances.find("pose_array") != displayInstances.end())
        {
            displayInstances["pose_array"]->processMessage(std::make_shared<const geometry_msgs::msg::PoseArray>(msg->pose_array));
        }
        if (displayInstances.find("pose_with_covariance") != displayInstances.end())
        {
            displayInstances["pose_with_covariance"]->processMessage(std::make_shared<const geometry_msgs::msg::PoseWithCovarianceStamped>(msg->pose_with_covariance));
        }
        if (displayInstances.find("pose_with_covariance_raw") != displayInstances.end())
        {
            displayInstances["pose_with_covariance_raw"]->processMessage(std::make_shared<const geometry_msgs::msg::PoseWithCovariance>(msg->pose_with_covariance_raw));
        }
        if (displayInstances.find("path") != displayInstances.end())
        {
            displayInstances["path"]->processMessage(std::make_shared<const nav_msgs::msg::Path>(msg->path));
        }
        if (displayInstances.find("odometry") != displayInstances.end())
        {
            displayInstances["odometry"]->processMessage(std::make_shared<const nav_msgs::msg::Odometry>(msg->odometry));
        }
        if (displayInstances.find("map") != displayInstances.end())
        {
            displayInstances["map"]->processMessage(std::make_shared<const nav_msgs::msg::OccupancyGrid>(msg->map));
        }
        if (displayInstances.find("marker") != displayInstances.end())
        {
            displayInstances["marker"]->processMessage(std::make_shared<const visualization_msgs::msg::Marker>(msg->marker));
        }
        if (displayInstances.find("marker_array") != displayInstances.end())
        {
            displayInstances["marker_array"]->processMessage(std::make_shared<const visualization_msgs::msg::MarkerArray>(msg->marker_array));
        }
        if (displayInstances.find("laser_scan") != displayInstances.end())
        {
            displayInstances["laser_scan"]->processMessage(std::make_shared<const sensor_msgs::msg::LaserScan>(msg->laser_scan));
        }
        if (displayInstances.find("point_cloud2") != displayInstances.end())
        {
            displayInstances["point_cloud2"]->processMessage(std::make_shared<const sensor_msgs::msg::PointCloud2>(msg->point_cloud2));
        }
        if (displayInstances.find("range") != displayInstances.end())
        {
            displayInstances["range"]->processMessage(std::make_shared<const sensor_msgs::msg::Range>(msg->range));
        }
        if (displayInstances.find("twist") != displayInstances.end())
        {
            displayInstances["twist"]->processMessage(std::make_shared<const geometry_msgs::msg::TwistStamped>(msg->twist));
        }
        if (displayInstances.find("twist_raw") != displayInstances.end())
        {
            displayInstances["twist_raw"]->processMessage(std::make_shared<const geometry_msgs::msg::Twist>(msg->twist_raw));
        }
        if (displayInstances.find("wrench") != displayInstances.end())
        {
            displayInstances["wrench"]->processMessage(std::make_shared<const geometry_msgs::msg::WrenchStamped>(msg->wrench));
        }
        if (displayInstances.find("wrench_raw") != displayInstances.end())
        {
            displayInstances["wrench_raw"]->processMessage(std::make_shared<const geometry_msgs::msg::Wrench>(msg->wrench_raw));
        }
    }
};
#endif // CUSTOM_MSG_TEMPLATES_
