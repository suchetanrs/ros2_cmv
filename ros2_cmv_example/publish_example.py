import rclpy
import numpy as np
import struct
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point, PolygonStamped, AccelStamped, TwistStamped
from geometry_msgs.msg import Polygon, Point32, PoseArray, Pose, Quaternion, PoseStamped, WrenchStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry, OccupancyGrid, GridCells
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Range, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from ros2_cmv_example.msg import Example  # Import your custom message

class CustomMessagePublisher(Node):
    def __init__(self):
        super().__init__('custom_message_publisher')
        
        # Create a publisher for the custom message
        self.publisher = self.create_publisher(Example, 'custom_point_topic', 10)
        self.get_logger().info("CustomMessagePublisher node has been started.")
        
        # Timer to call the publishing function every 1 second
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        # Create a header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # Current timestamp
        header.frame_id = 'map'  # Set the frame to 'map'

        # Create an AccelStamped message
        accel_stamped = AccelStamped()
        accel_stamped.header = header

        # Set the linear acceleration
        accel_stamped.accel.linear.x = 1.0  # Example linear acceleration in x
        accel_stamped.accel.linear.y = 0.0  # Example linear acceleration in y
        accel_stamped.accel.linear.z = 0.5  # Example linear acceleration in z

        # Set the angular acceleration
        accel_stamped.accel.angular.x = 0.0  # Example angular acceleration in x
        accel_stamped.accel.angular.y = 0.1  # Example angular acceleration in y
        accel_stamped.accel.angular.z = 0.2  # Example angular acceleration in z

        # Create a CameraInfo message
        camera_info = CameraInfo()
        camera_info.header = header

        # Camera intrinsic parameters (example values)
        camera_info.height = 480  # Image height
        camera_info.width = 640  # Image width
        camera_info.distortion_model = 'plumb_bob'  # Standard distortion model

        # Distortion coefficients (example values for a simple pinhole camera)
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Assuming no distortion

        # Intrinsic camera matrix (example values)
        # | fx  0 cx |
        # |  0 fy cy |
        # |  0  0  1 |
        camera_info.k = [
            525.0, 0.0, 319.5,
            0.0, 525.0, 239.5,
            0.0, 0.0, 1.0
        ]

        # Rectification matrix (identity matrix for no rectification)
        camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # Projection matrix (example values, assuming no distortion)
        # | fx  0 cx  0 |
        # |  0 fy cy  0 |
        # |  0  0  1  0 |
        camera_info.p = [
            525.0, 0.0, 319.5, 0.0,
            0.0, 525.0, 239.5, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        # ROI (Region of Interest, optional)
        camera_info.roi.x_offset = 0
        camera_info.roi.y_offset = 0
        camera_info.roi.height = 0  # Full image
        camera_info.roi.width = 0   # Full image
        camera_info.roi.do_rectify = False

        # Create GridCells
        grid_cells = GridCells()
        grid_cells.header = header
        grid_cells.cell_width = 1.0  # Example width of each cell
        grid_cells.cell_height = 1.0  # Example height of each cell

        # Add points to the grid
        grid_cells.cells.append(Point(x=0.0, y=0.0, z=0.0))  # Example cell
        grid_cells.cells.append(Point(x=5.0, y=1.0, z=0.0))  # Example cell
        grid_cells.cells.append(Point(x=6.0, y=2.0, z=0.0))  # Example cell

        # Create point1
        point1 = PointStamped()
        point1.header = header
        point1.point = Point(x=0.0, y=0.0, z=3.0)  # Sample coordinates for point1

        # Create point2
        point2 = PointStamped()
        point2.header = header
        point2.point = Point(x=0.0, y=0.0, z=-3.0)  # Sample coordinates for point2

        # Create a polygon1 (PolygonStamped)
        polygon1 = PolygonStamped()
        polygon1.header = header

        # Define the points of the polygon (in this case, a triangle)
        polygon_points = [
            Point32(x=0.0, y=0.0, z=0.0),
            Point32(x=1.0, y=0.0, z=0.0),
            Point32(x=0.5, y=1.0, z=0.0)
        ]
        
        polygon1.polygon = Polygon(points=polygon_points)  # Assign points to the polygon

        # Create a PoseArray (for multiple poses)
        pose_array = PoseArray()
        pose_array.header = header

        # Define some poses (you can add more poses as needed)
        pose1 = Pose()
        pose1.position.x = 1.0
        pose1.position.y = 2.0
        pose1.position.z = 3.0
        pose1.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # Identity quaternion
        
        pose2 = Pose()
        pose2.position.x = -1.0
        pose2.position.y = -2.0
        pose2.position.z = 3.0
        pose2.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # Identity quaternion
        
        pose_array.poses = [pose1, pose2]  # Add poses to the PoseArray

        # Create a Path message
        path = Path()
        path.header = header

        # Define some poses for the path
        path_pose1 = PoseStamped()
        path_pose1.header = header
        path_pose1.pose.position.x = 0.0
        path_pose1.pose.position.y = 0.0
        path_pose1.pose.position.z = 0.0
        path_pose1.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        path_pose2 = PoseStamped()
        path_pose2.header = header
        path_pose2.pose.position.x = 1.0
        path_pose2.pose.position.y = 1.0
        path_pose2.pose.position.z = 0.0
        path_pose2.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        path_pose3 = PoseStamped()
        path_pose3.header = header
        path_pose3.pose.position.x = 2.0
        path_pose3.pose.position.y = 0.5
        path_pose3.pose.position.z = 0.0
        path_pose3.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        path.poses = [path_pose1, path_pose2, path_pose3]  # Add PoseStamped to the path

        # Create PointCloud2
        pointcloud = PointCloud2()
        pointcloud.header = header

        # Define PointCloud2 fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pointcloud.fields = fields
        pointcloud.is_bigendian = False
        pointcloud.point_step = 12  # Each point has 3 fields (x, y, z), each 4 bytes
        pointcloud.row_step = pointcloud.point_step * 3  # 3 points in a single row
        pointcloud.is_dense = True
        pointcloud.height = 1  # Single row
        pointcloud.width = 3  # 3 points

        # Define PointCloud2 data
        points = [
            [3.0, -2.0, 3.0],
            [4.0, -2.0, 3.0],
            [5.0, -2.0, 0.0],
        ]
        flattened_points = np.array(points, dtype=np.float32).flatten()
        pointcloud.data = struct.pack(f'{len(flattened_points)}f', *flattened_points)

        self.pcl_publisher = self.create_publisher(PointCloud2, 'pcl_scan', 10)
        self.pcl_publisher.publish(pointcloud)

        # Create an Odometry message
        odometry = Odometry()
        odometry.header = header
        odometry.child_frame_id = 'base_link'  # Set the child frame

        # Set the pose in the odometry message
        odometry.pose.pose.position.x = 1.0
        odometry.pose.pose.position.y = 2.0
        odometry.pose.pose.position.z = 0.0
        odometry.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Set the twist in the odometry message
        odometry.twist.twist.linear.x = 0.5
        odometry.twist.twist.linear.y = 0.0
        odometry.twist.twist.linear.z = 0.0
        odometry.twist.twist.angular.x = 0.0
        odometry.twist.twist.angular.y = 0.0
        odometry.twist.twist.angular.z = 0.1

        # Create an OccupancyGrid message
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header = header

        # Set the metadata for the map
        occupancy_grid.info.map_load_time = self.get_clock().now().to_msg()  # Time the map was loaded
        occupancy_grid.info.resolution = 0.1  # Map resolution (meters/cell)
        occupancy_grid.info.width = 10       # Number of cells in the x direction
        occupancy_grid.info.height = 10      # Number of cells in the y direction
        occupancy_grid.info.origin.position.x = 3.0  # Origin of the map
        occupancy_grid.info.origin.position.y = 3.0
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Fill the map with some dummy data (-1 for unknown, 0 for free, 100 for occupied)
        occupancy_grid.data = [
            0 if (i + j) % 2 == 0 else 100
            for j in range(occupancy_grid.info.height)
            for i in range(occupancy_grid.info.width)
        ]

        # Create a LaserScan message
        laser_scan = LaserScan()
        laser_scan.header = header

        # Set LaserScan parameters
        laser_scan.angle_min = -1.57  # Start angle of the scan [radians]
        laser_scan.angle_max = 1.57   # End angle of the scan [radians]
        laser_scan.angle_increment = 0.01  # Angle between measurements [radians]
        laser_scan.time_increment = 0.0  # Time between measurements [seconds]
        laser_scan.scan_time = 0.1  # Time between scans [seconds]
        laser_scan.range_min = 0.2  # Minimum range value [meters]
        laser_scan.range_max = 10.0  # Maximum range value [meters]

        # Populate the ranges array (dummy data for illustration)
        num_readings = int((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment)
        laser_scan.ranges = [1.0 for _ in range(num_readings)]  # Example distances [meters]
        laser_scan.intensities = [100.0 for _ in range(num_readings)]  # Example intensities

        # Create Range
        range_msg = Range()
        range_msg.header = header
        range_msg.radiation_type = Range.ULTRASOUND  # Choose ULTRASOUND or INFRARED
        range_msg.field_of_view = 0.785398  # 45 degrees in radians
        range_msg.min_range = 0.2  # Minimum measurable range in meters
        range_msg.max_range = 4.0  # Maximum measurable range in meters
        range_msg.range = 2.5  # Current measured range in meters

        wrench_stamped_msg = WrenchStamped()
        wrench_stamped_msg.header = header

        # Populate force components
        wrench_stamped_msg.wrench.force.x = 10.0  # Force in X direction
        wrench_stamped_msg.wrench.force.y = 5.0   # Force in Y direction
        wrench_stamped_msg.wrench.force.z = 2.0   # Force in Z direction

        # Populate torque components
        wrench_stamped_msg.wrench.torque.x = 1.0  # Torque around X axis
        wrench_stamped_msg.wrench.torque.y = 0.5  # Torque around Y axis
        wrench_stamped_msg.wrench.torque.z = 0.2  # Torque around Z axis

        self.get_logger().info(f'Publishing wrench stamped: Frame({wrench_stamped_msg.header.frame_id}), '
                            f'Force({wrench_stamped_msg.wrench.force.x}, {wrench_stamped_msg.wrench.force.y}, {wrench_stamped_msg.wrench.force.z}), '
                            f'Torque({wrench_stamped_msg.wrench.torque.x}, {wrench_stamped_msg.wrench.torque.y}, {wrench_stamped_msg.wrench.torque.z})')

        # Create a single Marker
        marker = Marker()
        marker.header = header
        marker.ns = "example_namespace"  # Namespace for the marker
        marker.id = 1  # Unique ID for the marker
        marker.type = Marker.SPHERE  # Type of marker (e.g., SPHERE, CUBE, etc.)
        marker.action = Marker.ADD  # Action to add the marker

        # Set the marker position
        marker.pose.position.x = 1.0
        marker.pose.position.y = 2.0
        marker.pose.position.z = 3.0

        # Set the marker orientation (default is 0 for a simple marker)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set the marker scale (size)
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Set the marker color
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 0.0  # Blue
        marker.color.a = 1.0  # Alpha (transparency, 1.0 = opaque)

        marker_array = MarkerArray()

        # First marker (red)
        marker1 = Marker()
        marker1.header = header
        marker1.ns = "array_namespace"
        marker1.id = 2
        marker1.type = Marker.CUBE
        marker1.action = Marker.ADD
        marker1.pose.position.x = 2.0
        marker1.pose.position.y = 3.0
        marker1.pose.position.z = 1.0
        marker1.scale.x = 0.5
        marker1.scale.y = 0.5
        marker1.scale.z = 0.5
        marker1.color.r = 1.0
        marker1.color.g = 0.0
        marker1.color.b = 0.0
        marker1.color.a = 1.0
        marker_array.markers.append(marker1)

        # Second marker (green)
        marker2 = Marker()
        marker2.header = header
        marker2.ns = "array_namespace"
        marker2.id = 3
        marker2.type = Marker.CYLINDER
        marker2.action = Marker.ADD
        marker2.pose.position.x = 4.0
        marker2.pose.position.y = 1.0
        marker2.pose.position.z = 0.5
        marker2.scale.x = 0.3
        marker2.scale.y = 0.3
        marker2.scale.z = 1.0
        marker2.color.r = 0.0
        marker2.color.g = 1.0
        marker2.color.b = 0.0
        marker2.color.a = 1.0
        marker_array.markers.append(marker2)

        # Third marker (blue)
        marker3 = Marker()
        marker3.header = header
        marker3.ns = "array_namespace"
        marker3.id = 4
        marker3.type = Marker.ARROW
        marker3.action = Marker.ADD
        marker3.pose.position.x = -1.0
        marker3.pose.position.y = -2.0
        marker3.pose.position.z = 0.0
        marker3.scale.x = 0.5
        marker3.scale.y = 0.1
        marker3.scale.z = 0.1
        marker3.color.r = 0.0
        marker3.color.g = 0.0
        marker3.color.b = 1.0
        marker3.color.a = 1.0
        marker_array.markers.append(marker3)

        pose_stamped = PoseStamped()
        pose_stamped.header = header

        # Set position
        pose_stamped.pose.position.x = 7.0
        pose_stamped.pose.position.y = 5.0
        pose_stamped.pose.position.z = 0.0

        # Set orientation (example: facing forward)
        pose_stamped.pose.orientation.x = 0.3
        pose_stamped.pose.orientation.y = 0.23
        pose_stamped.pose.orientation.z = 0.4
        pose_stamped.pose.orientation.w = 1.0

        pose_with_covariance_stamped = PoseWithCovarianceStamped()
        pose_with_covariance_stamped.header = header

        # Set position
        pose_with_covariance_stamped.pose.pose.position.x = 6.0
        pose_with_covariance_stamped.pose.pose.position.y = 7.0
        pose_with_covariance_stamped.pose.pose.position.z = 0.0

        # Set orientation
        pose_with_covariance_stamped.pose.pose.orientation.x = 0.0
        pose_with_covariance_stamped.pose.pose.orientation.y = 0.0
        pose_with_covariance_stamped.pose.pose.orientation.z = 0.0
        pose_with_covariance_stamped.pose.pose.orientation.w = 1.0

        # Set covariance (example values, 6x6 covariance matrix)
        pose_with_covariance_stamped.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        # Create a TwistStamped message
        twist_stamped = TwistStamped()
        twist_stamped.header = header

        # Set linear velocity
        twist_stamped.twist.linear.x = -1.0  # Linear velocity in x-direction
        twist_stamped.twist.linear.y = -0.5  # Linear velocity in y-direction
        twist_stamped.twist.linear.z = 0.0  # Linear velocity in z-direction

        # Set angular velocity
        twist_stamped.twist.angular.x = 0.0  # Angular velocity around x-axis
        twist_stamped.twist.angular.y = 0.0  # Angular velocity around y-axis
        twist_stamped.twist.angular.z = 0.3  # Angular velocity around z-axis

        # Create a custom message and populate it
        custom_msg = Example()
        custom_msg.header = header
        custom_msg.accel = accel_stamped
        custom_msg.camera_info = camera_info
        custom_msg.grid_cells = grid_cells
        custom_msg.point1 = point1
        custom_msg.point2 = point2
        custom_msg.polygon1 = polygon1
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
        custom_msg.pose_with_covariance = pose_with_covariance_stamped
        custom_msg.twist = twist_stamped
        custom_msg.wrench = wrench_stamped_msg

        # Log the message for debugging
        self.get_logger().info(f"Publishing CustomMessage with Point1: {point1.point} and Point2: {point2.point}")
        self.get_logger().info(f"Polygon with {len(polygon_points)} points")
        self.get_logger().info(f"PoseArray with {len(pose_array.poses)} poses")
        self.get_logger().info(f"Path with {len(path.poses)} poses")
        self.get_logger().info(f"Odometry with linear velocity: {odometry.twist.twist.linear.x}")
        self.get_logger().info(f"Map with {len(occupancy_grid.data)} cells")
        self.get_logger().info(f"LaserScan with {len(laser_scan.ranges)} ranges")
        self.get_logger().info(f"PointCloud2 with {len(pointcloud.data)} points")
        self.get_logger().info(f"Range with range: {range_msg.range}")

        # Publish the custom message
        self.publisher.publish(custom_msg)
        # Publish the laser scan message separately
        self.laser_scan_publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.laser_scan_publisher.publish(laser_scan)

        self.grid_cell_publisher = self.create_publisher(GridCells, 'grid_cells', 10)
        self.grid_cell_publisher.publish(grid_cells)


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
