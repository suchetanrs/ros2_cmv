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

#include "custom_msg_visualizer/cmv_macros.hpp"
#include INCLUDE_PROJECT_HEADER(custom_msg_process.hpp)

namespace MESSAGE_NAME
{
    void processCustomMessage(const CustomMessage::ConstSharedPtr &msg, std::unordered_map<std::string, std::shared_ptr<custom_msg_visualizer::IExposedDisplay>> &enabledInstances)
    {
        custom_msg_visualizer::globalValues.setHeader(msg->header);
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
        if (enabledInstances.find("point") != enabledInstances.end())
        {
            enabledInstances["point"]->processMessage(std::make_shared<const geometry_msgs::msg::PointStamped>(msg->point));
        }
        if (enabledInstances.find("point_raw") != enabledInstances.end())
        {
            enabledInstances["point_raw"]->processMessage(std::make_shared<const geometry_msgs::msg::Point>(msg->point_raw));
        }
        if (enabledInstances.find("polygon") != enabledInstances.end())
        {
            enabledInstances["polygon"]->processMessage(std::make_shared<const geometry_msgs::msg::PolygonStamped>(msg->polygon));
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

    void processCustomMessageVectors(const CustomMessage::ConstSharedPtr &msg, std::shared_ptr<ArrayMessageAssist> arrayAssist)
    {
        arrayAssist->processArray<geometry_msgs::msg::AccelStamped>(msg->accel_vector, "accel_vector", "geometry_msgs/AccelStamped");
        arrayAssist->processArray<geometry_msgs::msg::Accel>(msg->accel_raw_vector, "accel_raw_vector", "geometry_msgs/Accel");
        arrayAssist->processArray<sensor_msgs::msg::CameraInfo>(msg->camera_info_vector, "camera_info_vector", "sensor_msgs/CameraInfo");
        arrayAssist->processArray<nav_msgs::msg::GridCells>(msg->grid_cells_vector, "grid_cells_vector", "nav_msgs/GridCells");
        arrayAssist->processArray<geometry_msgs::msg::PointStamped>(msg->point_vector, "point_vector", "geometry_msgs/PointStamped");
        arrayAssist->processArray<geometry_msgs::msg::Point>(msg->point_raw_vector, "point_raw_vector", "geometry_msgs/Point");
        arrayAssist->processArray<geometry_msgs::msg::PolygonStamped>(msg->polygon_vector, "polygon_vector", "geometry_msgs/PolygonStamped");
        arrayAssist->processArray<geometry_msgs::msg::Polygon>(msg->polygon_raw_vector, "polygon_raw_vector", "geometry_msgs/Polygon");
        arrayAssist->processArray<geometry_msgs::msg::PoseStamped>(msg->pose_vector, "pose_vector", "geometry_msgs/PoseStamped");
        arrayAssist->processArray<geometry_msgs::msg::Pose>(msg->pose_raw_vector, "pose_raw_vector", "geometry_msgs/Pose");
        arrayAssist->processArray<geometry_msgs::msg::PoseArray>(msg->pose_array_vector, "pose_array_vector", "geometry_msgs/PoseArray");
        arrayAssist->processArray<geometry_msgs::msg::PoseWithCovarianceStamped>(msg->pose_with_covariance_vector, "pose_with_covariance_vector", "geometry_msgs/PoseWithCovarianceStamped");
        arrayAssist->processArray<geometry_msgs::msg::PoseWithCovariance>(msg->pose_with_covariance_raw_vector, "pose_with_covariance_raw_vector", "geometry_msgs/PoseWithCovariance");
        arrayAssist->processArray<nav_msgs::msg::Path>(msg->path_vector, "path_vector", "nav_msgs/Path");
        arrayAssist->processArray<nav_msgs::msg::Odometry>(msg->odometry_vector, "odometry_vector", "nav_msgs/Odometry");
        arrayAssist->processArray<nav_msgs::msg::OccupancyGrid>(msg->map_vector, "map_vector", "nav_msgs/OccupancyGrid");
        arrayAssist->processArray<visualization_msgs::msg::Marker>(msg->marker_vector, "marker_vector", "visualization_msgs/Marker");
        arrayAssist->processArray<visualization_msgs::msg::MarkerArray>(msg->marker_array_vector, "marker_array_vector", "visualization_msgs/MarkerArray");
        arrayAssist->processArray<sensor_msgs::msg::LaserScan>(msg->laser_scan_vector, "laser_scan_vector", "sensor_msgs/LaserScan");
        arrayAssist->processArray<sensor_msgs::msg::PointCloud2>(msg->point_cloud2_vector, "point_cloud2_vector", "sensor_msgs/PointCloud2");
        arrayAssist->processArray<sensor_msgs::msg::Range>(msg->range_vector, "range_vector", "sensor_msgs/Range");
        arrayAssist->processArray<geometry_msgs::msg::TwistStamped>(msg->twist_vector, "twist_vector", "geometry_msgs/TwistStamped");
        arrayAssist->processArray<geometry_msgs::msg::Twist>(msg->twist_raw_vector, "twist_raw_vector", "geometry_msgs/Twist");
        arrayAssist->processArray<geometry_msgs::msg::WrenchStamped>(msg->wrench_vector, "wrench_vector", "geometry_msgs/WrenchStamped");
        arrayAssist->processArray<geometry_msgs::msg::Wrench>(msg->wrench_raw_vector, "wrench_raw_vector", "geometry_msgs/Wrench");
    }
};