#ifndef CUSTOM_MSG_METADATA_HPP_
#define CUSTOM_MSG_METADATA_HPP_

#include <string>
#include <vector>
#include "ros2_cmv_cmake_example/msg/example_c_make.hpp"

using CustomMessage = ros2_cmv_cmake_example::msg::ExampleCMake;

static std::vector<std::string> variableTypes = {
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

static std::vector<std::string> variableNames = {
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

#endif // CUSTOM_MSG_METADATA_HPP_
