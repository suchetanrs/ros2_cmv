#ifndef CUSTOM_MSG_TEMPLATES_
#define CUSTOM_MSG_TEMPLATES_

#include "ros2_cmv_example/msg/example.hpp"
#include "ros2_cmv/exposed_displays.hpp"

using CustomMessage = ros2_cmv_example::msg::Example;

static const std::vector<std::string> variableTypes = {
    // "std_msgs/Header",
    "geometry_msgs/AccelStamped",
    "sensor_msgs/CameraInfo",
    "nav_msgs/GridCells",
    "geometry_msgs/PointStamped",
    "geometry_msgs/PointStamped",
    "geometry_msgs/PolygonStamped",
    "geometry_msgs/PoseStamped",
    "geometry_msgs/PoseArray",
    "geometry_msgs/PoseWithCovarianceStamped",
    "nav_msgs/Path",
    "nav_msgs/Odometry",
    "nav_msgs/OccupancyGrid",
    "visualization_msgs/Marker",
    "visualization_msgs/MarkerArray",
    "sensor_msgs/LaserScan",
    "sensor_msgs/PointCloud2",
    "sensor_msgs/Range",
    "geometry_msgs/TwistStamped",
    "geometry_msgs/WrenchStamped"};

static const std::vector<std::string> variableNames = {
    // "header",
    "accel",
    "camera_info",
    "grid_cells",
    "point1",
    "point2",
    "polygon1",
    "pose",
    "pose_array",
    "pose_with_covariance",
    "path",
    "odometry",
    "map",
    "marker",
    "marker_array",
    "laser_scan",
    "point_cloud2",
    "range",
    "twist",
    "wrench"};

inline void processCustomMessage(const CustomMessage::ConstSharedPtr &msg, std::unordered_map<std::string, std::shared_ptr<IExposedDisplay>>& displayInstances)
{
    displayInstances["accel"]->processMessage(std::make_shared<const geometry_msgs::msg::AccelStamped>(msg->accel));
    displayInstances["camera_info"]->processMessage(std::make_shared<const sensor_msgs::msg::CameraInfo>(msg->camera_info));
    displayInstances["grid_cells"]->processMessage(std::make_shared<const nav_msgs::msg::GridCells>(msg->grid_cells));
    displayInstances["point1"]->processMessage(std::make_shared<const geometry_msgs::msg::PointStamped>(msg->point1));
    displayInstances["point2"]->processMessage(std::make_shared<const geometry_msgs::msg::PointStamped>(msg->point2));
    displayInstances["polygon1"]->processMessage(std::make_shared<const geometry_msgs::msg::PolygonStamped>(msg->polygon1));
    displayInstances["pose"]->processMessage(std::make_shared<const geometry_msgs::msg::PoseStamped>(msg->pose));
    displayInstances["pose_array"]->processMessage(std::make_shared<const geometry_msgs::msg::PoseArray>(msg->pose_array));
    displayInstances["pose_with_covariance"]->processMessage(std::make_shared<const geometry_msgs::msg::PoseWithCovarianceStamped>(msg->pose_with_covariance));
    displayInstances["path"]->processMessage(std::make_shared<const nav_msgs::msg::Path>(msg->path));
    displayInstances["odometry"]->processMessage(std::make_shared<const nav_msgs::msg::Odometry>(msg->odometry));
    displayInstances["map"]->processMessage(std::make_shared<const nav_msgs::msg::OccupancyGrid>(msg->map));
    displayInstances["marker"]->processMessage(std::make_shared<const visualization_msgs::msg::Marker>(msg->marker));
    displayInstances["marker_array"]->processMessage(std::make_shared<const visualization_msgs::msg::MarkerArray>(msg->marker_array));
    displayInstances["laser_scan"]->processMessage(std::make_shared<const sensor_msgs::msg::LaserScan>(msg->laser_scan));
    displayInstances["point_cloud2"]->processMessage(std::make_shared<const sensor_msgs::msg::PointCloud2>(msg->point_cloud2));
    displayInstances["range"]->processMessage(std::make_shared<const sensor_msgs::msg::Range>(msg->range));
    displayInstances["twist"]->processMessage(std::make_shared<const geometry_msgs::msg::TwistStamped>(msg->twist));
    displayInstances["wrench"]->processMessage(std::make_shared<const geometry_msgs::msg::WrenchStamped>(msg->wrench));
}

#endif // CUSTOM_MSG_TEMPLATES_