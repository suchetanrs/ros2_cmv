/**
    Copyright 2025 Suchetan Saravanan.

    Licensed to the Apache Software Foundation (ASF) under one
    or more contributor license agreements.  See the NOTICE file
    distributed with this work for additional information
    regarding copyright ownership.  The ASF licenses this file
    to you under the Apache License, Version 2.0 (the
    "License"); you may not use this file except in compliance
    with the License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing,
    software distributed under the License is distributed on an
    "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
    KIND, either express or implied.  See the License for the
    specific language governing permissions and limitations
    under the License.
*/

#ifndef CUSTOM_MSG_METADATA_HPP_
#define CUSTOM_MSG_METADATA_HPP_

#include "custom_msg_visualizer_msgs/msg/all_types.hpp"
#include <vector>
#include <string>

// This is just an example having all the supported message types.
// In the actual generated code, this file will be replaced with the specified message and the message fields.

using CustomMessage = custom_msg_visualizer_msgs::msg::AllTypes;

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

static std::vector<std::string> variableTypesArrays = {
    "geometry_msgs/Pose",
    "geometry_msgs/PoseWithCovariance"
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

static std::vector<std::string> variableNamesArrays = {
    "pose_custom_array",
    "pose_with_covariance_array",
};
#endif // CUSTOM_MSG_METADATA_HPP_
