#!/usr/bin/env python3
"""
Node that publishes a custom message containing various ROS 2 message types.
"""

import struct

import numpy as np
import rclpy
from rclpy.node import Node

# Standard ROS messages
from std_msgs.msg import Header
from geometry_msgs.msg import (
    Accel, AccelStamped, Twist, TwistStamped, Wrench, WrenchStamped,
    Pose, PoseStamped, PoseArray, PoseWithCovariance, PoseWithCovarianceStamped,
    Quaternion, Point, PointStamped, Polygon, PolygonStamped, Point32, Vector3
)
from nav_msgs.msg import Path, Odometry, OccupancyGrid, GridCells
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Range, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray

# Custom message
# from ros2_cmv_example_cmake.msg import Example
from ros2_cmv_msgs.msg import AllTypes


class CustomMessagePublisher(Node):
    """Node that publishes a custom ROS message containing various message types."""

    def __init__(self):
        super().__init__('custom_message_publisher')

        # Create publishers
        self.custom_msg_publisher = self.create_publisher(AllTypes, 'example_msg_topic', 10)

        self.get_logger().info("CustomMessagePublisher node has been started.")

        # Timer to call the publishing function every 1 second
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        """Construct and publish a custom message containing multiple ROS messages."""

        # Create a header with current time and frame
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        # --------------------------
        # Accel/AccelStamped
        # --------------------------
        accel_stamped = AccelStamped(header=header)
        accel_stamped.accel.linear.x = 1.0
        accel_stamped.accel.linear.y = 0.0
        accel_stamped.accel.linear.z = 0.5
        accel_stamped.accel.angular.x = 0.0
        accel_stamped.accel.angular.y = 0.1
        accel_stamped.accel.angular.z = 0.2

        accel = Accel()
        accel.linear.x = 1.0
        accel.linear.y = 0.0
        accel.linear.z = 0.5
        accel.angular.x = 0.0
        accel.angular.y = 0.1
        accel.angular.z = 0.2

        # --------------------------
        # CameraInfo
        # --------------------------
        camera_info = CameraInfo(header=header)
        camera_info.height = 480
        camera_info.width = 640
        camera_info.distortion_model = 'plumb_bob'
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.k = [525.0, 0.0,   319.5,
                         0.0,   525.0, 239.5,
                         0.0,   0.0,   1.0]
        camera_info.r = [1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0]
        camera_info.p = [525.0, 0.0,   319.5, 0.0,
                         0.0,   525.0, 239.5, 0.0,
                         0.0,   0.0,   1.0,   0.0]

        # --------------------------
        # GridCells
        # --------------------------
        grid_cells = GridCells(header=header)
        grid_cells.cell_width = 1.0
        grid_cells.cell_height = 1.0
        grid_cells.cells.append(Point(x=0.0, y=0.0, z=0.0))
        grid_cells.cells.append(Point(x=5.0, y=1.0, z=0.0))
        grid_cells.cells.append(Point(x=6.0, y=2.0, z=0.0))

        # --------------------------
        # PointStamped (point1, point2)
        # --------------------------
        point1 = PointStamped(header=header, point=Point(x=0.0, y=0.0, z=3.0))
        point2 = PointStamped(header=header, point=Point(x=0.0, y=0.0, z=-3.0))
        point_raw = Point(x=10.0, y=10.0, z=10.0)

        # --------------------------
        # Polygon/PolygonStamped
        # --------------------------
        polygon_points = [
            Point32(x=0.0, y=0.0, z=0.0),
            Point32(x=1.0, y=0.0, z=0.0),
            Point32(x=0.5, y=1.0, z=0.0)
        ]
        polygon1 = PolygonStamped(header=header, polygon=Polygon(points=polygon_points))

        polygon_points2 = [
            Point32(x=3.0, y=0.0, z=4.0),
            Point32(x=5.0, y=0.0, z=4.0),
            Point32(x=4.0, y=1.0, z=4.0)
        ]
        polygon_raw = Polygon(points=polygon_points2)

        # --------------------------
        # PoseArray
        # --------------------------
        pose_array = PoseArray(header=header)
        pose1 = Pose(
            position=Point(x=1.0, y=2.0, z=3.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        pose2 = Pose(
            position=Point(x=-1.0, y=-2.0, z=3.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        pose_array.poses = [pose1, pose2]

        # --------------------------
        # Path
        # --------------------------
        path = Path(header=header)
        path_pose1 = PoseStamped(
            header=header,
            pose=Pose(position=Point(x=0.0, y=0.0, z=0.0),
                      orientation=Quaternion())
        )
        path_pose2 = PoseStamped(
            header=header,
            pose=Pose(position=Point(x=1.0, y=1.0, z=0.0),
                      orientation=Quaternion())
        )
        path_pose3 = PoseStamped(
            header=header,
            pose=Pose(position=Point(x=2.0, y=0.5, z=0.0),
                      orientation=Quaternion())
        )
        path.poses = [path_pose1, path_pose2, path_pose3]

        # --------------------------
        # PointCloud2
        # --------------------------
        pointcloud = PointCloud2(header=header)
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        pointcloud.fields = fields
        pointcloud.is_bigendian = False
        pointcloud.point_step = 12
        pointcloud.height = 1
        pointcloud.width = 3
        pointcloud.row_step = pointcloud.point_step * pointcloud.width
        pointcloud.is_dense = True

        # Define PointCloud2 data (3 points)
        points = [
            [3.0, -2.0, 3.0],
            [4.0, -2.0, 3.0],
            [5.0, -2.0, 0.0]
        ]
        flattened_points = np.array(points, dtype=np.float32).flatten()
        pointcloud.data = struct.pack(f'{len(flattened_points)}f', *flattened_points)

        # Publish separately below (we keep the instance here)

        # --------------------------
        # Odometry
        # --------------------------
        odometry = Odometry(header=header, child_frame_id='base_link')
        odometry.pose.pose.position.x = 1.0
        odometry.pose.pose.position.y = 2.0
        odometry.pose.pose.orientation.w = 1.0
        odometry.twist.twist.linear.x = 0.5
        odometry.twist.twist.angular.z = 0.1

        # --------------------------
        # OccupancyGrid
        # --------------------------
        occupancy_grid = OccupancyGrid(header=header)
        occupancy_grid.info.map_load_time = self.get_clock().now().to_msg()
        occupancy_grid.info.resolution = 0.1
        occupancy_grid.info.width = 10
        occupancy_grid.info.height = 10
        occupancy_grid.info.origin.position.x = 3.0
        occupancy_grid.info.origin.position.y = 3.0
        occupancy_grid.info.origin.orientation.w = 1.0
        occupancy_grid.data = [
            0 if (i + j) % 2 == 0 else 100
            for j in range(occupancy_grid.info.height)
            for i in range(occupancy_grid.info.width)
        ]

        # --------------------------
        # LaserScan
        # --------------------------
        laser_scan = LaserScan(header=header)
        laser_scan.angle_min = -1.57
        laser_scan.angle_max = 1.57
        laser_scan.angle_increment = 0.01
        laser_scan.scan_time = 0.1
        laser_scan.range_min = 0.2
        laser_scan.range_max = 10.0
        num_readings = int((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment)
        laser_scan.ranges = [1.0] * num_readings
        laser_scan.intensities = [100.0] * num_readings

        # --------------------------
        # Range
        # --------------------------
        range_msg = Range(header=header)
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.785398  # ~45 degrees
        range_msg.min_range = 0.2
        range_msg.max_range = 4.0
        range_msg.range = 2.5

        # --------------------------
        # Wrench/WrenchStamped
        # --------------------------
        wrench_stamped_msg = WrenchStamped(header=header)
        wrench_stamped_msg.wrench.force.x = 10.0
        wrench_stamped_msg.wrench.force.y = 5.0
        wrench_stamped_msg.wrench.force.z = 2.0
        wrench_stamped_msg.wrench.torque.x = 1.0
        wrench_stamped_msg.wrench.torque.y = 0.5
        wrench_stamped_msg.wrench.torque.z = 0.2

        wrench_raw = Wrench()
        wrench_raw.force.x = 5.0
        wrench_raw.force.y = 2.0
        wrench_raw.force.z = 1.0
        wrench_raw.torque.x = 1.0
        wrench_raw.torque.y = 0.5
        wrench_raw.torque.z = 0.2

        # --------------------------
        # Marker / MarkerArray
        # --------------------------
        marker = Marker(
            header=header,
            ns="example_namespace",
            id=1,
            type=Marker.SPHERE,
            action=Marker.ADD,
            pose=Pose(position=Point(x=1.0, y=2.0, z=3.0),
                      orientation=Quaternion()),
            scale=Vector3(x=1.0, y=1.0, z=1.0),
            color=self._make_color(r=1.0, g=0.0, b=0.0, a=1.0)
        )

        marker_array = MarkerArray()
        # Marker 1 (red cube)
        marker1 = Marker(
            header=header,
            ns="array_namespace",
            id=2,
            type=Marker.CUBE,
            action=Marker.ADD,
            pose=Pose(position=Point(x=2.0, y=3.0, z=1.0),
                      orientation=Quaternion()),
            scale=Vector3(x=0.5, y=0.5, z=0.5),
            color=self._make_color(r=1.0, g=0.0, b=0.0, a=1.0)
        )
        marker_array.markers.append(marker1)

        # Marker 2 (green cylinder)
        marker2 = Marker(
            header=header,
            ns="array_namespace",
            id=3,
            type=Marker.CYLINDER,
            action=Marker.ADD,
            pose=Pose(position=Point(x=4.0, y=1.0, z=0.5),
                      orientation=Quaternion()),
            scale=Vector3(x=0.3, y=0.3, z=1.0),
            color=self._make_color(r=0.0, g=1.0, b=0.0, a=1.0)
        )
        marker_array.markers.append(marker2)

        # Marker 3 (blue arrow)
        marker3 = Marker(
            header=header,
            ns="array_namespace",
            id=4,
            type=Marker.ARROW,
            action=Marker.ADD,
            pose=Pose(position=Point(x=-1.0, y=-2.0, z=0.0),
                      orientation=Quaternion()),
            scale=Vector3(x=0.5, y=0.1, z=0.1),
            color=self._make_color(r=0.0, g=0.0, b=1.0, a=1.0)
        )
        marker_array.markers.append(marker3)

        # --------------------------
        # Pose / PoseStamped / PoseWithCovariance / PoseWithCovarianceStamped
        # --------------------------
        pose_stamped = PoseStamped(
            header=header,
            pose=Pose(
                position=Point(x=7.0, y=5.0, z=0.0),
                orientation=Quaternion(x=0.3, y=0.23, z=0.4, w=1.0)
            )
        )

        pose_raw = Pose(
            position=Point(x=8.0, y=6.0, z=-3.0),
            orientation=Quaternion()  # Identity orientation
        )

        pose_with_covariance_stamped = PoseWithCovarianceStamped(header=header)
        pose_with_covariance_stamped.pose.pose.position.x = 6.0
        pose_with_covariance_stamped.pose.pose.position.y = 7.0
        pose_with_covariance_stamped.pose.pose.orientation.w = 1.0
        pose_with_covariance_stamped.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        pose_with_covariance = PoseWithCovariance()
        pose_with_covariance.pose.position.x = 5.0
        pose_with_covariance.pose.position.y = 8.0
        pose_with_covariance.pose.position.z = -1.0
        pose_with_covariance.pose.orientation.w = 1.0
        pose_with_covariance.covariance = [
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ]

        # --------------------------
        # Twist / TwistStamped
        # --------------------------
        twist_stamped = TwistStamped(header=header)
        twist_stamped.twist.linear.x = -1.0
        twist_stamped.twist.linear.y = -0.5
        twist_stamped.twist.linear.z = 0.0
        twist_stamped.twist.angular.z = 0.3

        twist = Twist()
        twist.linear.x = 5.5
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 1.0
        twist.angular.z = 1.2

        # --------------------------
        # Custom Message
        # --------------------------
        custom_msg = AllTypes()
        custom_msg.header = header
        custom_msg.accel = accel_stamped
        custom_msg.accel_raw = accel
        custom_msg.camera_info = camera_info
        custom_msg.grid_cells = grid_cells
        custom_msg.point1 = point1
        custom_msg.point2 = point2
        custom_msg.point_raw = point_raw
        custom_msg.polygon1 = polygon1
        custom_msg.polygon_raw = polygon_raw
        custom_msg.pose_array = pose_array
        custom_msg.path = path
        custom_msg.odometry = odometry
        custom_msg.map = occupancy_grid
        custom_msg.laser_scan = laser_scan
        custom_msg.point_cloud2 = pointcloud
        custom_msg.range = range_msg
        custom_msg.marker = marker
        custom_msg.marker_array = marker_array
        custom_msg.pose = pose_stamped
        custom_msg.pose_raw = pose_raw
        custom_msg.pose_with_covariance = pose_with_covariance_stamped
        custom_msg.pose_with_covariance_raw = pose_with_covariance
        custom_msg.twist = twist_stamped
        custom_msg.twist_raw = twist
        custom_msg.wrench = wrench_stamped_msg
        # custom_msg.wrench_raw = wrench_raw

        # --------------------------
        # Logging (for debugging)
        # --------------------------
        # self.get_logger().info(
        #     f'Publishing WrenchStamped: Frame({wrench_stamped_msg.header.frame_id}), '
        #     f'Force({wrench_stamped_msg.wrench.force.x}, '
        #     f'{wrench_stamped_msg.wrench.force.y}, '
        #     f'{wrench_stamped_msg.wrench.force.z}), '
        #     f'Torque({wrench_stamped_msg.wrench.torque.x}, '
        #     f'{wrench_stamped_msg.wrench.torque.y}, '
        #     f'{wrench_stamped_msg.wrench.torque.z})'
        # )
        # self.get_logger().info(f"Publishing CustomMessage with Point1: {point1.point} and Point2: {point2.point}")
        # self.get_logger().info(f"Polygon with {len(polygon_points)} points")
        # self.get_logger().info(f"PoseArray with {len(pose_array.poses)} poses")
        # self.get_logger().info(f"Path with {len(path.poses)} poses")
        # self.get_logger().info(f"Odometry with linear velocity: {odometry.twist.twist.linear.x}")
        # self.get_logger().info(f"Map with {len(occupancy_grid.data)} cells")
        # self.get_logger().info(f"LaserScan with {len(laser_scan.ranges)} ranges")
        # self.get_logger().info(f"PointCloud2 has {pointcloud.width} points in 1 row")
        # self.get_logger().info(f"Range with value: {range_msg.range}")

        # --------------------------
        # Publish messages
        # --------------------------
        self.custom_msg_publisher.publish(custom_msg)
        self.get_logger().info("Published CustomMessage")

    @staticmethod
    def _make_color(r=0.0, g=0.0, b=0.0, a=1.0):
        """Helper function to return a color with RGBA channels."""
        from std_msgs.msg import ColorRGBA
        return ColorRGBA(r=r, g=g, b=b, a=a)


def main(args=None):
    rclpy.init(args=args)

    custom_message_publisher = CustomMessagePublisher()

    try:
        rclpy.spin(custom_message_publisher)
    except KeyboardInterrupt:
        pass

    # Clean up and shutdown
    custom_message_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()