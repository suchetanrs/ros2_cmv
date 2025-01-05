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
from custom_msg_visualizer_example.msg import ExampleCMake


class CustomMessagePublisher(Node):
    """Node that publishes a custom ROS message containing various message types."""

    def __init__(self):
        super().__init__('custom_message_publisher')

        # Create publishers
        self.custom_msg_publisher = self.create_publisher(ExampleCMake, 'all_types_topic', 10)

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
        custom_msg = ExampleCMake()
        custom_msg.header = header
        custom_msg.accel = accel_stamped
        custom_msg.accel_raw = accel
        custom_msg.camera_info = camera_info
        custom_msg.grid_cells = grid_cells
        custom_msg.point = point1
        custom_msg.point_raw = point_raw
        custom_msg.polygon = polygon1
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
        custom_msg.wrench_raw = wrench_raw

        # --------------------------
        # Vectors
        # --------------------------
        pose_array_msg = PoseArray()
        pose_array_msg.header = header
        for i in range(35):
            #
            # geometry_msgs/AccelStamped[] accel_vector
            #
            new_accel_stamped = AccelStamped()
            new_accel_stamped.header = header
            new_accel_stamped.accel.linear.x = float(i)
            new_accel_stamped.accel.angular.z = float(i) * 0.1
            # custom_msg.accel_vector.append(new_accel_stamped)

            #
            # geometry_msgs/Accel[] accel_raw_vector
            #
            new_accel_raw = Accel()
            new_accel_raw.linear.x = float(i) * 0.1
            new_accel_raw.angular.z = float(i) * 0.01
            # custom_msg.accel_raw_vector.append(new_accel_raw)

            #
            # sensor_msgs/CameraInfo[] camera_info_vector
            #
            new_camera_info = CameraInfo(header=header)
            new_camera_info.height = 480
            new_camera_info.width = 640
            new_camera_info.distortion_model = 'plumb_bob'
            new_camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            new_camera_info.k = [525.0, 0.0,   319.5,
                            0.0,   525.0, 239.5,
                            0.0,   0.0,   i]
            new_camera_info.r = [1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0]
            new_camera_info.p = [525.0, 0.0,   319.5, 0.0,
                            0.0,   525.0, 239.5, 0.0,
                            0.0,   0.0,   1.0,   0.0]
            custom_msg.camera_info_vector.append(new_camera_info)

            #
            # nav_msgs/GridCells[] grid_cells_vector
            #
            new_grid_cells = GridCells()
            new_grid_cells.header = header
            new_grid_cells.cell_width = 1.0
            new_grid_cells.cell_height = 1.0
            # Example: append a single point in each GridCells
            # new_grid_cells.cells.append(Point(x=float(i), y=0.0, z=0.0))
            # custom_msg.grid_cells_vector.append(new_grid_cells)

            #
            # geometry_msgs/PointStamped[] point_vector
            #
            new_point_stamped = PointStamped()
            new_point_stamped.header = header
            new_point_stamped.point.x = float(i)
            new_point_stamped.point.y = float(i) * 0.1
            custom_msg.point_vector.append(new_point_stamped)

            #
            # geometry_msgs/Point[] point_raw_vector
            #
            new_point_raw = Point()
            new_point_raw.x = float(i) + 0.5
            new_point_raw.y = -1.0
            new_point_raw.z = 2.0
            custom_msg.point_raw_vector.append(new_point_raw)

            #
            # geometry_msgs/PolygonStamped[] polygon_vector
            #
            new_polygon_stamped = PolygonStamped()
            new_polygon_stamped.header = header
            new_polygon_stamped.polygon.points.append(Point32(x=float(i), y=0.0, z=0.0))
            new_polygon_stamped.polygon.points.append(Point32(x=float(i) + 1.0, y=1.0, z=0.0))
            custom_msg.polygon_vector.append(new_polygon_stamped)

            #
            # geometry_msgs/Polygon[] polygon_raw_vector
            #
            new_polygon_raw = Polygon()
            # new_polygon_raw.points.append(Point32(x=float(i) * 0.1, y=0.1, z=0.0))
            # custom_msg.polygon_raw_vector.append(new_polygon_raw)

            #
            # geometry_msgs/PoseStamped[] pose_vector
            #
            new_pose_stamped = PoseStamped()
            new_pose_stamped.header = header
            new_pose_stamped.pose.position.x = float(i) * 0.1
            new_pose_stamped.pose.position.y = 0.0
            new_pose_stamped.pose.orientation.w = 1.0
            # custom_msg.pose_vector.append(new_pose_stamped)

            #
            # geometry_msgs/Pose[] pose_raw_vector
            #
            new_pose_raw = Pose()
            new_pose_raw.position.x = float(i) * 0.2
            new_pose_raw.position.y = -0.5
            new_pose_raw.orientation.w = 1.0
            # custom_msg.pose_raw_vector.append(new_pose_raw)

            #
            # geometry_msgs/PoseArray[] pose_array_vector
            #
            new_pose_array = PoseArray()
            new_pose_array.header = header
            # new_pose_array.poses.append(new_pose_stamped.pose)
            # new_pose_array.poses.append(new_pose_raw)
            # custom_msg.pose_array_vector.append(new_pose_array)

            #
            # geometry_msgs/PoseWithCovarianceStamped[] pose_with_covariance_vector
            #
            new_pose_cov_stamped = PoseWithCovarianceStamped()
            new_pose_cov_stamped.header = header
            new_pose_cov_stamped.pose.pose.position.x = float(i)
            new_pose_cov_stamped.pose.pose.orientation.w = 1.0
            new_pose_cov_stamped.pose.covariance = [
                0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.1
            ]
            custom_msg.pose_with_covariance_vector.append(new_pose_cov_stamped)

            #
            # geometry_msgs/PoseWithCovariance[] pose_with_covariance_raw_vector
            #
            new_pose_cov_raw = PoseWithCovariance()
            new_pose_cov_raw.pose.position.y = float(i) * 0.5
            new_pose_cov_raw.pose.orientation.w = 1.0
            new_pose_cov_raw.covariance = [
                1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0
            ]
            # custom_msg.pose_with_covariance_raw_vector.append(new_pose_cov_raw)

            #
            # nav_msgs/Path[] path_vector
            #
            new_path = Path()
            new_path.header = header
            new_path_pose = PoseStamped(header=header, pose=new_pose_raw)
            # new_path.poses.append(new_path_pose)
            # custom_msg.path_vector.append(new_path)

            #
            # nav_msgs/Odometry[] odometry_vector
            #
            new_odometry = Odometry()
            new_odometry.header = header
            new_odometry.child_frame_id = "base_link"
            new_odometry.pose.pose.position.x = float(i) * 0.05
            new_odometry.pose.pose.orientation.w = 1.0
            # custom_msg.odometry_vector.append(new_odometry)

            #
            # nav_msgs/OccupancyGrid[] map_vector
            #
            new_map = OccupancyGrid()
            new_map.header = header
            new_map.info.resolution = 0.2
            new_map.info.width = 5
            new_map.info.height = 5
            new_map.info.origin.orientation.w = 1.0
            # Fill each map with a pattern or zeros
            new_map.data = [0 if (x + y) % 2 == 0 else 100 for y in range(5) for x in range(5)]
            # custom_msg.map_vector.append(new_map)

            #
            # visualization_msgs/Marker[] marker_vector
            #
            new_marker = Marker()
            new_marker.header = header
            new_marker.id = i
            new_marker.type = Marker.SPHERE
            new_marker.pose.position.x = float(i) * 0.3
            new_marker.pose.orientation.w = 1.0
            new_marker.scale.x = 0.5
            new_marker.scale.y = 0.5
            new_marker.scale.z = 0.5
            new_marker.color = self._make_color(r=1.0, g=0.0, b=1.0, a=1.0)
            # custom_msg.marker_vector.append(new_marker)

            #
            # visualization_msgs/MarkerArray[] marker_array_vector
            #
            new_marker_array = MarkerArray()
            # new_marker_array.markers.append(new_marker)
            # custom_msg.marker_array_vector.append(new_marker_array)

            #
            # sensor_msgs/LaserScan[] laser_scan_vector
            #
            new_laser_scan = LaserScan()
            new_laser_scan.header = header
            new_laser_scan.angle_min = -1.57
            new_laser_scan.angle_max = 1.57
            new_laser_scan.angle_increment = 0.05
            new_laser_scan.range_min = 0.2
            new_laser_scan.range_max = 10.0
            num_samples = int((new_laser_scan.angle_max - new_laser_scan.angle_min) /
                            new_laser_scan.angle_increment)
            new_laser_scan.ranges = [2.0] * num_samples
            # custom_msg.laser_scan_vector.append(new_laser_scan)

            #
            # sensor_msgs/PointCloud2[] point_cloud2_vector
            #
            new_pointcloud2 = PointCloud2()
            new_pointcloud2.header = header
            new_pointcloud2.height = 1
            new_pointcloud2.width = 2
            new_pointcloud2.is_dense = True
            new_pointcloud2.is_bigendian = False
            new_pointcloud2.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            new_pointcloud2.point_step = 12
            new_pointcloud2.row_step = new_pointcloud2.point_step * new_pointcloud2.width
            # Minimal data example for 2 points
            minimal_points = [float(i), 0.0, 0.0,
                            float(i) + 1.0, 1.0, 1.0]
            new_pointcloud2.data = struct.pack('6f', *minimal_points)
            # custom_msg.point_cloud2_vector.append(new_pointcloud2)

            #
            # sensor_msgs/Range[] range_vector
            #
            new_range = Range()
            new_range.header = header
            new_range.radiation_type = Range.ULTRASOUND
            new_range.field_of_view = 0.785398  # ~45 deg
            new_range.min_range = 0.2
            new_range.max_range = 4.0
            new_range.range = 2.0 + (i * 0.01)
            # custom_msg.range_vector.append(new_range)

            #
            # geometry_msgs/TwistStamped[] twist_vector
            #
            new_twist_stamped = TwistStamped()
            new_twist_stamped.header = header
            new_twist_stamped.twist.linear.x = float(i) * 0.05
            new_twist_stamped.twist.angular.z = float(i) * 0.005
            # custom_msg.twist_vector.append(new_twist_stamped)

            #
            # geometry_msgs/Twist[] twist_raw_vector
            #
            new_twist_raw = Twist()
            new_twist_raw.linear.x = float(i)
            new_twist_raw.angular.y = float(i) * 0.1
            # custom_msg.twist_raw_vector.append(new_twist_raw)

            #
            # geometry_msgs/WrenchStamped[] wrench_vector
            #
            new_wrench_stamped = WrenchStamped()
            new_wrench_stamped.header = header
            new_wrench_stamped.wrench.force.x = float(i)
            new_wrench_stamped.wrench.torque.z = float(i) * 0.1
            # custom_msg.wrench_vector.append(new_wrench_stamped)

            #
            # geometry_msgs/Wrench[] wrench_raw_vector
            #
            new_wrench_raw = Wrench()
            new_wrench_raw.force.y = float(i) * 0.2
            new_wrench_raw.torque.x = float(i) * 0.05
            # custom_msg.wrench_raw_vector.append(new_wrench_raw)


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