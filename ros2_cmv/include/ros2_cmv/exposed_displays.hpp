#ifndef EXPOSED_DISPLAYS_HPP_
#define EXPOSED_DISPLAYS_HPP_

#if defined(ROS_DISTRO_JAZZY) || defined(ROS_DISTRO_ROLLING)
#include <rviz_default_plugins/displays/accel/accel_display.hpp>
#include <rviz_default_plugins/displays/camera_info/camera_info_display.hpp>
#include <rviz_default_plugins/displays/marker/marker_display.hpp>
#include <rviz_default_plugins/displays/marker_array/marker_array_display.hpp>
#include <rviz_default_plugins/displays/pose/pose_display.hpp>
#include <rviz_default_plugins/displays/pose_covariance/pose_with_covariance_display.hpp>
#include <rviz_default_plugins/displays/twist/twist_display.hpp>
#endif
// #include <rviz_default_plugins/displays/effort/effort_display.hpp>
#include <rviz_default_plugins/displays/grid_cells/grid_cells_display.hpp>
#include <rviz_default_plugins/displays/laser_scan/laser_scan_display.hpp>
#include <rviz_default_plugins/displays/map/map_display.hpp>
#include <rviz_default_plugins/displays/odometry/odometry_display.hpp>
#include <rviz_default_plugins/displays/path/path_display.hpp>
#include <rviz_default_plugins/displays/point/point_stamped_display.hpp>
#include <rviz_default_plugins/displays/pointcloud/point_cloud2_display.hpp>
#include <rviz_default_plugins/displays/polygon/polygon_display.hpp>
#include <rviz_default_plugins/displays/pose_array/pose_array_display.hpp>
#include <rviz_default_plugins/displays/range/range_display.hpp>
#include <rviz_default_plugins/displays/wrench/wrench_display.hpp>

#include "ros2_cmv/exposed_display_core.hpp"
#include "ros2_cmv/cmv_common.hpp" // for the header value

namespace ros2_cmv
{
#if defined(ROS_DISTRO_JAZZY) || defined(ROS_DISTRO_ROLLING)
    /////////////////////////////////////////////////////////////////////

    using ExposedAccelStampedType = ExposedDisplay<rviz_default_plugins::displays::AccelStampedDisplay, geometry_msgs::msg::AccelStamped::ConstSharedPtr>;
    class ExposedAccelStampedDisplay : public ExposedAccelStampedType
    {
    public:
        ExposedAccelStampedDisplay(rviz_common::DisplayContext *context) : ExposedAccelStampedType(context) {};
    };
    REGISTER_DISPLAY_CLASS("geometry_msgs/AccelStamped", ExposedAccelStampedDisplay)

    class ExposedAccelDisplay : public ExposedAccelStampedDisplay
    {
    public:
        ExposedAccelDisplay(rviz_common::DisplayContext *context) : ExposedAccelStampedDisplay(context) {};

        void processMessage(std::any param) override
        {
            auto accelStamped = std::make_shared<geometry_msgs::msg::AccelStamped>();
            accelStamped->header = globalValues.getHeader();
            accelStamped->accel = *(castMessage<geometry_msgs::msg::Accel::ConstSharedPtr>(param));
            geometry_msgs::msg::AccelStamped::ConstSharedPtr accelStampedConst = accelStamped;
            ExposedAccelStampedDisplay::processMessage(accelStampedConst);
        }
    };
    REGISTER_DISPLAY_CLASS("geometry_msgs/Accel", ExposedAccelDisplay)

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

    using ExposedPoseStampedType = ExposedDisplay<rviz_default_plugins::displays::PoseDisplay, geometry_msgs::msg::PoseStamped::ConstSharedPtr>;
    class ExposedPoseStampedDisplay : public ExposedPoseStampedType
    {
    public:
        ExposedPoseStampedDisplay(rviz_common::DisplayContext *context) : ExposedPoseStampedType(context) {};
    };
    REGISTER_DISPLAY_CLASS("geometry_msgs/PoseStamped", ExposedPoseStampedDisplay)

    class ExposedPoseDisplay : public ExposedPoseStampedDisplay
    {
    public:
        ExposedPoseDisplay(rviz_common::DisplayContext *context) : ExposedPoseStampedDisplay(context) {};

        void processMessage(std::any param) override
        {
            auto poseStamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
            poseStamped->header = globalValues.getHeader();
            poseStamped->pose = *(castMessage<geometry_msgs::msg::Pose::ConstSharedPtr>(param));
            geometry_msgs::msg::PoseStamped::ConstSharedPtr poseStampedConst = poseStamped;
            ExposedPoseStampedDisplay::processMessage(poseStampedConst);
        }
    };
    REGISTER_DISPLAY_CLASS("geometry_msgs/Pose", ExposedPoseDisplay)

    /////////////////////////////////////////////////////////////////////

    using ExposedPoseWithCovarianceType = ExposedDisplay<rviz_default_plugins::displays::PoseWithCovarianceDisplay, geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr>;
    class ExposedPoseWithCovarianceStampedDisplay : public ExposedPoseWithCovarianceType
    {
    public:
        ExposedPoseWithCovarianceStampedDisplay(rviz_common::DisplayContext *context) : ExposedPoseWithCovarianceType(context) {};
    };
    REGISTER_DISPLAY_CLASS("geometry_msgs/PoseWithCovarianceStamped", ExposedPoseWithCovarianceStampedDisplay)

    class ExposedPoseWithCovarianceDisplay : public ExposedPoseWithCovarianceStampedDisplay
    {
    public:
        explicit ExposedPoseWithCovarianceDisplay(rviz_common::DisplayContext *context) : ExposedPoseWithCovarianceStampedDisplay(context) {}

        void processMessage(std::any param) override
        {
            auto poseWithCovarianceStamped = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
            poseWithCovarianceStamped->header = globalValues.getHeader();
            poseWithCovarianceStamped->pose = *(castMessage<geometry_msgs::msg::PoseWithCovariance::ConstSharedPtr>(param));
            geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr poseWithCovarianceStampedConst = poseWithCovarianceStamped;
            ExposedPoseWithCovarianceStampedDisplay::processMessage(poseWithCovarianceStampedConst);
        }
    };

    REGISTER_DISPLAY_CLASS("geometry_msgs/PoseWithCovariance", ExposedPoseWithCovarianceDisplay)
    
    /////////////////////////////////////////////////////////////////////

    using ExposedTwistStampedType = ExposedDisplay<rviz_default_plugins::displays::TwistStampedDisplay, geometry_msgs::msg::TwistStamped::ConstSharedPtr>;
    class ExposedTwistStampedDisplay : public ExposedTwistStampedType
    {
    public:
        ExposedTwistStampedDisplay(rviz_common::DisplayContext *context) : ExposedTwistStampedType(context) {};
    };
    REGISTER_DISPLAY_CLASS("geometry_msgs/TwistStamped", ExposedTwistStampedDisplay)

    class ExposedTwistDisplay : public ExposedTwistStampedDisplay
    {
    public:
        explicit ExposedTwistDisplay(rviz_common::DisplayContext *context) : ExposedTwistStampedDisplay(context) {}

        void processMessage(std::any param) override
        {
            auto twistStamped = std::make_shared<geometry_msgs::msg::TwistStamped>();
            twistStamped->header = globalValues.getHeader();
            twistStamped->twist = *(castMessage<geometry_msgs::msg::Twist::ConstSharedPtr>(param));
            geometry_msgs::msg::TwistStamped::ConstSharedPtr twistStampedConst = twistStamped;
            ExposedTwistStampedDisplay::processMessage(twistStampedConst);
        }
    };

    REGISTER_DISPLAY_CLASS("geometry_msgs/Twist", ExposedTwistDisplay)

#endif
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

    class ExposedPointDisplay : public ExposedPointStampedDisplay
    {
    public:
        ExposedPointDisplay(rviz_common::DisplayContext *context) : ExposedPointStampedDisplay(context) {};

        void processMessage(std::any param) override
        {
            auto pointStamped = std::make_shared<geometry_msgs::msg::PointStamped>();
            pointStamped->header = globalValues.getHeader();
            pointStamped->point = *(castMessage<geometry_msgs::msg::Point::ConstSharedPtr>(param));
            geometry_msgs::msg::PointStamped::ConstSharedPtr pointStampedConst = pointStamped;
            ExposedPointStampedDisplay::processMessage(pointStampedConst);
        }
    };
    REGISTER_DISPLAY_CLASS("geometry_msgs/Point", ExposedPointDisplay)

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

    class ExposedPolygonDisplay : public ExposedPolygonStampedDisplay
    {
    public:
        ExposedPolygonDisplay(rviz_common::DisplayContext *context) : ExposedPolygonStampedDisplay(context) {};

        void processMessage(std::any param) override
        {
            auto polygonStamped = std::make_shared<geometry_msgs::msg::PolygonStamped>();
            polygonStamped->header = globalValues.getHeader();
            polygonStamped->polygon = *(castMessage<geometry_msgs::msg::Polygon::ConstSharedPtr>(param));
            geometry_msgs::msg::PolygonStamped::ConstSharedPtr polygonStampedConst = polygonStamped;
            ExposedPolygonStampedDisplay::processMessage(polygonStampedConst);
        }
    };
    REGISTER_DISPLAY_CLASS("geometry_msgs/Polygon", ExposedPolygonDisplay)

    /////////////////////////////////////////////////////////////////////

    using ExposedPoseArrayType = ExposedDisplay<rviz_default_plugins::displays::PoseArrayDisplay, geometry_msgs::msg::PoseArray::ConstSharedPtr>;
    class ExposedPoseArrayDisplay : public ExposedPoseArrayType
    {
    public:
        ExposedPoseArrayDisplay(rviz_common::DisplayContext *context) : ExposedPoseArrayType(context) {};
    };
    REGISTER_DISPLAY_CLASS("geometry_msgs/PoseArray", ExposedPoseArrayDisplay)

    /////////////////////////////////////////////////////////////////////

    using ExposedRangeType = ExposedDisplay<rviz_default_plugins::displays::RangeDisplay, sensor_msgs::msg::Range::ConstSharedPtr>;
    class ExposedRangeDisplay : public ExposedRangeType
    {
    public:
        ExposedRangeDisplay(rviz_common::DisplayContext *context) : ExposedRangeType(context) {};
    };
    REGISTER_DISPLAY_CLASS("sensor_msgs/Range", ExposedRangeDisplay)

    /////////////////////////////////////////////////////////////////////

    using ExposedWrenchType = ExposedDisplay<rviz_default_plugins::displays::WrenchDisplay, geometry_msgs::msg::WrenchStamped::ConstSharedPtr>;
    class ExposedWrenchStampedDisplay : public ExposedWrenchType
    {
    public:
        ExposedWrenchStampedDisplay(rviz_common::DisplayContext *context) : ExposedWrenchType(context) {};
    };
    REGISTER_DISPLAY_CLASS("geometry_msgs/WrenchStamped", ExposedWrenchStampedDisplay)

    class ExposedWrenchDisplay : public ExposedWrenchStampedDisplay
    {
    public:
        explicit ExposedWrenchDisplay(rviz_common::DisplayContext *context) : ExposedWrenchStampedDisplay(context) {}

        void processMessage(std::any param) override
        {
            auto wrenchStamped = std::make_shared<geometry_msgs::msg::WrenchStamped>();
            wrenchStamped->header = globalValues.getHeader();
            wrenchStamped->wrench = *(castMessage<geometry_msgs::msg::Wrench::ConstSharedPtr>(param));

            geometry_msgs::msg::WrenchStamped::ConstSharedPtr wrenchStampedConst = wrenchStamped;
            ExposedWrenchStampedDisplay::processMessage(wrenchStampedConst);
        }
    };
    REGISTER_DISPLAY_CLASS("geometry_msgs/Wrench", ExposedWrenchDisplay)
};

#endif // EXPOSED_DISPLAYS_HPP_