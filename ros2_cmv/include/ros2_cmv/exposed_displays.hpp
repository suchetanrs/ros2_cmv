#ifndef EXPOSED_POINT_STAMPED_DISPLAY_HPP_
#define EXPOSED_POINT_STAMPED_DISPLAY_HPP_

#include <rviz_default_plugins/displays/accel/accel_display.hpp>
#include <rviz_default_plugins/displays/camera_info/camera_info_display.hpp>
// #include <rviz_default_plugins/displays/effort/effort_display.hpp>
#include <rviz_default_plugins/displays/grid_cells/grid_cells_display.hpp>
#include <rviz_default_plugins/displays/laser_scan/laser_scan_display.hpp>
#include <rviz_default_plugins/displays/map/map_display.hpp>
#include <rviz_default_plugins/displays/marker/marker_display.hpp>
#include <rviz_default_plugins/displays/marker_array/marker_array_display.hpp>
#include <rviz_default_plugins/displays/odometry/odometry_display.hpp>
#include <rviz_default_plugins/displays/path/path_display.hpp>
#include <rviz_default_plugins/displays/point/point_stamped_display.hpp>
#include <rviz_default_plugins/displays/pointcloud/point_cloud2_display.hpp>
#include <rviz_default_plugins/displays/polygon/polygon_display.hpp>
#include <rviz_default_plugins/displays/pose/pose_display.hpp>
#include <rviz_default_plugins/displays/pose_array/pose_array_display.hpp>
#include <rviz_default_plugins/displays/pose_covariance/pose_with_covariance_display.hpp>
#include <rviz_default_plugins/displays/range/range_display.hpp>
#include <rviz_default_plugins/displays/twist/twist_display.hpp>
#include <rviz_default_plugins/displays/wrench/wrench_display.hpp>

#include "ros2_cmv/exposed_display_core.hpp"

/////////////////////////////////////////////////////////////////////

using ExposedAccelStampedType = ExposedDisplay<rviz_default_plugins::displays::AccelStampedDisplay, geometry_msgs::msg::AccelStamped::ConstSharedPtr>;
class ExposedAccelStampedDisplay : public ExposedAccelStampedType
{
public:
    ExposedAccelStampedDisplay(rviz_common::DisplayContext *context) : ExposedAccelStampedType(context) {};
};
REGISTER_DISPLAY_CLASS("geometry_msgs/AccelStamped", ExposedAccelStampedDisplay)

/////////////////////////////////////////////////////////////////////

using ExposedCameraInfoType = ExposedDisplay<rviz_default_plugins::displays::CameraInfoDisplay, sensor_msgs::msg::CameraInfo::ConstSharedPtr>;
class ExposedCameraInfoDisplay : public ExposedCameraInfoType
{
public:
    ExposedCameraInfoDisplay(rviz_common::DisplayContext *context) : ExposedCameraInfoType(context) {};
};
REGISTER_DISPLAY_CLASS("sensor_msgs/CameraInfo", ExposedCameraInfoDisplay)

// class ExposedEffortDisplay : public
// ExposedDisplay<rviz_default_plugins::displays::EffortDisplay>
// {
// public:
//     ExposedEffortDisplay(rviz_common::DisplayContext *context)
//         :
//         ExposedDisplay<rviz_default_plugins::displays::EffortDisplay>(context)
//         {};
// };

/////////////////////////////////////////////////////////////////////

using ExposedGridCellsType = ExposedDisplay<rviz_default_plugins::displays::GridCellsDisplay, nav_msgs::msg::GridCells::ConstSharedPtr>;
class ExposedGridCellsDisplay : public ExposedGridCellsType
{
public:
    ExposedGridCellsDisplay(rviz_common::DisplayContext *context) : ExposedGridCellsType(context) {};
};
REGISTER_DISPLAY_CLASS("nav_msgs/GridCells", ExposedGridCellsDisplay)

/////////////////////////////////////////////////////////////////////

using ExposedLaserScanType = ExposedDisplay<rviz_default_plugins::displays::LaserScanDisplay, sensor_msgs::msg::LaserScan::ConstSharedPtr>;
class ExposedLaserScanDisplay : public ExposedLaserScanType
{
public:
    ExposedLaserScanDisplay(rviz_common::DisplayContext *context) : ExposedLaserScanType(context) {};
};
REGISTER_DISPLAY_CLASS("sensor_msgs/LaserScan", ExposedLaserScanDisplay)

/////////////////////////////////////////////////////////////////////

using ExposedMapType = ExposedDisplay<rviz_default_plugins::displays::MapDisplay, nav_msgs::msg::OccupancyGrid::ConstSharedPtr>;
class ExposedMapDisplay : public ExposedMapType
{
public:
    ExposedMapDisplay(rviz_common::DisplayContext *context) : ExposedMapType(context) {};
};
REGISTER_DISPLAY_CLASS("nav_msgs/OccupancyGrid", ExposedMapDisplay)

/////////////////////////////////////////////////////////////////////

using ExposedMarkerType = ExposedDisplay<rviz_default_plugins::displays::MarkerDisplay, visualization_msgs::msg::Marker::ConstSharedPtr>;
class ExposedMarkerDisplay : public ExposedMarkerType
{
public:
    ExposedMarkerDisplay(rviz_common::DisplayContext *context) : ExposedMarkerType(context) {};
};
REGISTER_DISPLAY_CLASS("visualization_msgs/Marker", ExposedMarkerDisplay)

/////////////////////////////////////////////////////////////////////

using ExposedMarkerArrayType = ExposedDisplay<rviz_default_plugins::displays::MarkerArrayDisplay, visualization_msgs::msg::MarkerArray::ConstSharedPtr>;
class ExposedMarkerArrayDisplay : public ExposedMarkerArrayType
{
public:
    ExposedMarkerArrayDisplay(rviz_common::DisplayContext *context) : ExposedMarkerArrayType(context) {};
};
REGISTER_DISPLAY_CLASS("visualization_msgs/MarkerArray", ExposedMarkerArrayDisplay)

/////////////////////////////////////////////////////////////////////

using ExposedOdometryType = ExposedDisplay<rviz_default_plugins::displays::OdometryDisplay, nav_msgs::msg::Odometry::ConstSharedPtr>;
class ExposedOdometryDisplay : public ExposedOdometryType
{
public:
    ExposedOdometryDisplay(rviz_common::DisplayContext *context) : ExposedOdometryType(context) {};
};
REGISTER_DISPLAY_CLASS("nav_msgs/Odometry", ExposedOdometryDisplay)

/////////////////////////////////////////////////////////////////////

using ExposedPathType = ExposedDisplay<rviz_default_plugins::displays::PathDisplay, nav_msgs::msg::Path::ConstSharedPtr>;
class ExposedPathDisplay : public ExposedPathType
{
public:
    ExposedPathDisplay(rviz_common::DisplayContext *context) : ExposedPathType(context) {};
};
REGISTER_DISPLAY_CLASS("nav_msgs/Path", ExposedPathDisplay)

/////////////////////////////////////////////////////////////////////

using ExposedPointStampedType = ExposedDisplay<rviz_default_plugins::displays::PointStampedDisplay, geometry_msgs::msg::PointStamped::ConstSharedPtr>;
class ExposedPointStampedDisplay : public ExposedPointStampedType
{
public:
    ExposedPointStampedDisplay(rviz_common::DisplayContext *context) : ExposedPointStampedType(context) {};
};
REGISTER_DISPLAY_CLASS("geometry_msgs/PointStamped", ExposedPointStampedDisplay)

/////////////////////////////////////////////////////////////////////

using ExposedPointCloud2Type = ExposedDisplay<rviz_default_plugins::displays::PointCloud2Display, sensor_msgs::msg::PointCloud2::ConstSharedPtr>;
class ExposedPointCloud2Display : public ExposedPointCloud2Type
{
public:
    ExposedPointCloud2Display(rviz_common::DisplayContext *context) : ExposedPointCloud2Type(context) {};
};
REGISTER_DISPLAY_CLASS("sensor_msgs/PointCloud2", ExposedPointCloud2Display)

/////////////////////////////////////////////////////////////////////

using ExposedPolygonStampedType = ExposedDisplay<rviz_default_plugins::displays::PolygonDisplay, geometry_msgs::msg::PolygonStamped::ConstSharedPtr>;
class ExposedPolygonStampedDisplay : public ExposedPolygonStampedType
{
public:
    ExposedPolygonStampedDisplay(rviz_common::DisplayContext *context) : ExposedPolygonStampedType(context) {};
};
REGISTER_DISPLAY_CLASS("geometry_msgs/PolygonStamped", ExposedPolygonStampedDisplay)

/////////////////////////////////////////////////////////////////////

using ExposedPoseType = ExposedDisplay<rviz_default_plugins::displays::PoseDisplay, geometry_msgs::msg::PoseStamped::ConstSharedPtr>;
class ExposedPoseDisplay : public ExposedPoseType
{
public:
    ExposedPoseDisplay(rviz_common::DisplayContext *context) : ExposedPoseType(context) {};
};
REGISTER_DISPLAY_CLASS("geometry_msgs/PoseStamped", ExposedPoseDisplay)

/////////////////////////////////////////////////////////////////////

using ExposedPoseArrayType = ExposedDisplay<rviz_default_plugins::displays::PoseArrayDisplay, geometry_msgs::msg::PoseArray::ConstSharedPtr>;
class ExposedPoseArrayDisplay : public ExposedPoseArrayType
{
public:
    ExposedPoseArrayDisplay(rviz_common::DisplayContext *context) : ExposedPoseArrayType(context) {};
};
REGISTER_DISPLAY_CLASS("geometry_msgs/PoseArray", ExposedPoseArrayDisplay)

/////////////////////////////////////////////////////////////////////

using ExposedPoseWithCovarianceType = ExposedDisplay<rviz_default_plugins::displays::PoseWithCovarianceDisplay, geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr>;
class ExposedPoseWithCovarianceDisplay : public ExposedPoseWithCovarianceType
{
public:
    ExposedPoseWithCovarianceDisplay(rviz_common::DisplayContext *context) : ExposedPoseWithCovarianceType(context) {};
};
REGISTER_DISPLAY_CLASS("geometry_msgs/PoseWithCovarianceStamped", ExposedPoseWithCovarianceDisplay)

/////////////////////////////////////////////////////////////////////

using ExposedRangeType = ExposedDisplay<rviz_default_plugins::displays::RangeDisplay, sensor_msgs::msg::Range::ConstSharedPtr>;
class ExposedRangeDisplay : public ExposedRangeType
{
public:
    ExposedRangeDisplay(rviz_common::DisplayContext *context) : ExposedRangeType(context) {};
};
REGISTER_DISPLAY_CLASS("sensor_msgs/Range", ExposedRangeDisplay)

/////////////////////////////////////////////////////////////////////

using ExposedTwistStampedType = ExposedDisplay<rviz_default_plugins::displays::TwistStampedDisplay, geometry_msgs::msg::TwistStamped::ConstSharedPtr>;
class ExposedTwistStampedDisplay : public ExposedTwistStampedType
{
public:
    ExposedTwistStampedDisplay(rviz_common::DisplayContext *context) : ExposedTwistStampedType(context) {};
};
REGISTER_DISPLAY_CLASS("geometry_msgs/TwistStamped", ExposedTwistStampedDisplay)

/////////////////////////////////////////////////////////////////////

using ExposedWrenchType = ExposedDisplay<rviz_default_plugins::displays::WrenchDisplay, geometry_msgs::msg::WrenchStamped::ConstSharedPtr>;
class ExposedWrenchDisplay : public ExposedWrenchType
{
public:
    ExposedWrenchDisplay(rviz_common::DisplayContext *context) : ExposedWrenchType(context) {};
};
REGISTER_DISPLAY_CLASS("geometry_msgs/WrenchStamped", ExposedWrenchDisplay)

#endif // EXPOSED_POINT_STAMPED_DISPLAY_HPP_