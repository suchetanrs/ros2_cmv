#ifndef CUSTOM_MSG_TEMPLATES_
#define CUSTOM_MSG_TEMPLATES_

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

#include "ros2_cmv_example/msg/example.hpp"
#include "ros2_cmv/exposed_displays.hpp"

using CustomMessage = ros2_cmv_example::msg::Example;

static const std::vector<std::string> variableTypes = {
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
    "geometry_msgs/WrenchStamped",
};

static const std::vector<std::string> variableNames = {
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
    "wrench",
};

inline void processCustomMessage(const CustomMessage::ConstSharedPtr &msg, std::unordered_map<std::string, std::shared_ptr<IExposedDisplay>>& displayInstances)
{
    if(displayInstances.find("accel") != displayInstances.end()) {
        displayInstances["accel"]->processMessage(std::make_shared<const geometry_msgs::msg::AccelStamped>(msg->accel));
    }
    if(displayInstances.find("camera_info") != displayInstances.end()) {
        displayInstances["camera_info"]->processMessage(std::make_shared<const sensor_msgs::msg::CameraInfo>(msg->camera_info));
    }
    if(displayInstances.find("grid_cells") != displayInstances.end()) {
        displayInstances["grid_cells"]->processMessage(std::make_shared<const nav_msgs::msg::GridCells>(msg->grid_cells));
    }
    if(displayInstances.find("point1") != displayInstances.end()) {
        displayInstances["point1"]->processMessage(std::make_shared<const geometry_msgs::msg::PointStamped>(msg->point1));
    }
    if(displayInstances.find("point2") != displayInstances.end()) {
        displayInstances["point2"]->processMessage(std::make_shared<const geometry_msgs::msg::PointStamped>(msg->point2));
    }
    if(displayInstances.find("polygon1") != displayInstances.end()) {
        displayInstances["polygon1"]->processMessage(std::make_shared<const geometry_msgs::msg::PolygonStamped>(msg->polygon1));
    }
    if(displayInstances.find("pose") != displayInstances.end()) {
        displayInstances["pose"]->processMessage(std::make_shared<const geometry_msgs::msg::PoseStamped>(msg->pose));
    }
    if(displayInstances.find("pose_array") != displayInstances.end()) {
        displayInstances["pose_array"]->processMessage(std::make_shared<const geometry_msgs::msg::PoseArray>(msg->pose_array));
    }
    if(displayInstances.find("pose_with_covariance") != displayInstances.end()) {
        displayInstances["pose_with_covariance"]->processMessage(std::make_shared<const geometry_msgs::msg::PoseWithCovarianceStamped>(msg->pose_with_covariance));
    }
    if(displayInstances.find("path") != displayInstances.end()) {
        displayInstances["path"]->processMessage(std::make_shared<const nav_msgs::msg::Path>(msg->path));
    }
    if(displayInstances.find("odometry") != displayInstances.end()) {
        displayInstances["odometry"]->processMessage(std::make_shared<const nav_msgs::msg::Odometry>(msg->odometry));
    }
    if(displayInstances.find("map") != displayInstances.end()) {
        displayInstances["map"]->processMessage(std::make_shared<const nav_msgs::msg::OccupancyGrid>(msg->map));
    }
    if(displayInstances.find("marker") != displayInstances.end()) {
        displayInstances["marker"]->processMessage(std::make_shared<const visualization_msgs::msg::Marker>(msg->marker));
    }
    if(displayInstances.find("marker_array") != displayInstances.end()) {
        displayInstances["marker_array"]->processMessage(std::make_shared<const visualization_msgs::msg::MarkerArray>(msg->marker_array));
    }
    if(displayInstances.find("laser_scan") != displayInstances.end()) {
        displayInstances["laser_scan"]->processMessage(std::make_shared<const sensor_msgs::msg::LaserScan>(msg->laser_scan));
    }
    if(displayInstances.find("point_cloud2") != displayInstances.end()) {
        displayInstances["point_cloud2"]->processMessage(std::make_shared<const sensor_msgs::msg::PointCloud2>(msg->point_cloud2));
    }
    if(displayInstances.find("range") != displayInstances.end()) {
        displayInstances["range"]->processMessage(std::make_shared<const sensor_msgs::msg::Range>(msg->range));
    }
    if(displayInstances.find("twist") != displayInstances.end()) {
        displayInstances["twist"]->processMessage(std::make_shared<const geometry_msgs::msg::TwistStamped>(msg->twist));
    }
    if(displayInstances.find("wrench") != displayInstances.end()) {
        displayInstances["wrench"]->processMessage(std::make_shared<const geometry_msgs::msg::WrenchStamped>(msg->wrench));
    }
}

#endif // CUSTOM_MSG_TEMPLATES_
