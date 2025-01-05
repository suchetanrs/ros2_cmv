#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import math

# Import your custom message
from custom_msg_visualizer_example.msg import MapObstaclePoses

# Import standard and common ROS2 message types we need
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray

class MapObstaclePosesPublisher(Node):
    def __init__(self):
        super().__init__('example_cmake_publisher')

        # Create a publisher for our custom message type
        self.publisher_ = self.create_publisher(
            MapObstaclePoses,
            'map_robot_cov',
            10
        )

        # Publish at 1 Hz
        self.timer_ = self.create_timer(1.0, self.timer_callback)

        # For repeated runs, only generate once
        self.occupancy_grid = self.generate_occupancy_grid()
        # self.pose_with_covariance_array = self.generate_pose_with_covariances()  # Circle
        self.robot_path = self.generate_robot_path()  # Path in the maze
        self.marker_array = self.generate_obstacle_markers(self.occupancy_grid)

    def timer_callback(self):
        """
        Timer callback that publishes the MapObstaclePoses message.
        """
        msg = MapObstaclePoses()

        # Fill in the header of the main message if desired
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # 1) Occupancy Grid
        msg.map = self.occupancy_grid

        # 2) Pose With Covariance Array (Circle)
        # msg.pose_with_covariance_raw_vector = self.pose_with_covariance_array

        # 3) Path Poses with Covariance (Robot’s path in the maze)
        #   This field is assumed to exist in your .msg file, e.g.:
        #     geometry_msgs/PoseWithCovariance[] pose_with_cov_vector
        msg.pose_with_covariance_raw_vector = self.robot_path

        # 4) Marker Array around obstacles
        msg.marker_array = self.marker_array

        # Publish
        self.publisher_.publish(msg)
        self.get_logger().info(
            'Published MapObstaclePoses message with OccupancyGrid, Poses, Path, and MarkerArray.'
        )

    def generate_occupancy_grid(self):
        """
        Hard-code a 10x10 occupancy grid to form a simple “maze.”
        Cells with value=100 are obstacles, and cells with value=0 are free.
        """
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'

        # Setup metadata (resolution, width, height, origin)
        meta = MapMetaData()
        meta.resolution = 1.0  # 1 meter per cell
        meta.width = 10
        meta.height = 10
        meta.origin.position.x = 0.0
        meta.origin.position.y = 0.0
        meta.origin.orientation.w = 1.0
        grid.info = meta

        # Hard-coded maze layout (10×10), 100 = obstacle, 0 = free
        maze_data_2d = [
            [100, 100,   0,   0,   0, 100, 100, 100, 100, 100],
            [100,   0,   0, 100,   0,   0,   0,   0,   0, 100],
            [100,   0, 100, 100, 100,   0, 100, 100,   0, 100],
            [100,   0, 100,   0,   0,   0, 100,   0,   0, 100],
            [100,   0, 100,   0, 100, 100, 100,   0, 100, 100],
            [100,   0, 100,   0,   0,   0, 100,   0,   0, 100],
            [100,   0, 100, 100, 100, 100, 100,   0, 100, 100],
            [100,   0,   0,   0,   0,   0,   0,   0,   0, 100],
            [100, 100, 100, 100, 100, 100,   0, 100,   0, 100],
            [100, 100, 100, 100, 100, 100, 100, 100, 100, 100],
        ]

        # Flatten the 2D maze layout into a single 1D list in row-major order
        grid_data = []
        for row in maze_data_2d:
            for cell_value in row:
                grid_data.append(cell_value)
        grid.data = grid_data
        return grid

    def generate_pose_with_covariances(self):
        """
        Generate an array of PoseWithCovariance in a circle of radius 3.0.
        (Used to demonstrate the original example of a ring of poses)
        """
        poses = []
        num_poses = 8
        radius = 3.0
        for i in range(num_poses):
            angle = (2.0 * math.pi / num_poses) * i

            pwc = PoseWithCovariance()
            # Position in a circle
            pwc.pose.position.x = radius * math.cos(angle)
            pwc.pose.position.y = radius * math.sin(angle)
            pwc.pose.position.z = 0.0

            # Orientation facing outward
            pwc.pose.orientation.z = math.sin(angle / 2.0)
            pwc.pose.orientation.w = math.cos(angle / 2.0)

            # Example covariance (just a diagonal for illustration)
            pwc.covariance = [
                1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0
            ]
            poses.append(pwc)

        return poses

    def generate_robot_path(self):
        """
        Return a set of PoseWithCovariance that represents a path 
        navigating through the maze. Adjust the (x, y) coordinates 
        to valid free cells in your map. We'll place them roughly 
        at cell-centers in row/column style.
        """
        # Example path (x, y) in meters, assuming each cell is 1m and origin is (0,0).
        # Adjust these coordinates to fit your maze's free corridors.
        path_positions = [
            (1.5, 1.5),
            (2.5, 1.5),
            (2.5, 0.5),
            (3.5, 0.5),
            (4.5, 0.5),
            (4.5, 1.5),
            (5.5, 1.5),
            (6.5, 1.5),
            (7.5, 1.5),
            (8.5, 1.5),
            (8.5, 2.5),
            (8.5, 3.5),
            (7.5, 3.5),
            (7.5, 4.5),
        ]

        path_yaws = [
            0.0,
            -1.57,
            0.0,
            0.0,
            1.57,
            0.0,
            0.0,
            0.0,
            0.0,
            1.57,
            1.57,
            3.14,
            1.57,
            1.57,
        ]

        poses = []
        for i, (px, py) in enumerate(path_positions):
            pwc = PoseWithCovariance()
            pwc.pose.position.x = px
            pwc.pose.position.y = py
            pwc.pose.position.z = 0.0

            # Simple approach: make the orientation face "east" for horizontal moves,
            # then turn "north" for vertical. This is purely illustrative.
            # Alternatively, you could set a smoother heading or leave it as identity.
            if i < len(path_positions):
                next_px, next_py = path_positions[i]
                dx = next_px - px
                dy = next_py - py
                # yaw = math.atan2(dy, dx)
                yaw = path_yaws[i]
                # Convert yaw to quaternion (only around z-axis)
                # qz = sin(yaw/2), qw = cos(yaw/2)
                pwc.pose.orientation.z = math.sin(yaw / 2.0)
                pwc.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                # For the final pose, just reuse previous orientation
                pwc.pose.orientation.z = 0.0
                pwc.pose.orientation.w = 1.0

            # Example covariance (just a diagonal for illustration)
            pwc.covariance = [
                0.1*(i+0.5), 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.1*(i+0.5), 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            ]
            poses.append(pwc)

        return poses

    def generate_obstacle_markers(self, occupancy_grid):
        """
        Generate a MarkerArray for the cells that are obstacles (value=100).
        Each cell will be represented by a small cube marker.
        """
        marker_array = MarkerArray()
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height

        idx = 0
        marker_id = 0
        for row in range(height):
            for col in range(width):
                cell_value = occupancy_grid.data[idx]
                idx += 1
                if cell_value == 100:
                    marker = Marker()
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.header.frame_id = 'map'
                    marker.ns = 'obstacle_markers'
                    marker.id = marker_id
                    marker_id += 1

                    # We use CUBE for obstacles
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD

                    # Position the marker at the center of the cell
                    marker.pose.position.x = float(col) + 0.5
                    marker.pose.position.y = float(row) + 0.5
                    marker.pose.position.z = 0.5  # slightly above ground
                    marker.pose.orientation.w = 1.0

                    # Each cell is 1x1, so let's make the cube 1m
                    marker.scale.x = 0.5
                    marker.scale.y = 0.5
                    marker.scale.z = min(0.05 * idx, 2.0)

                    # Color (red) with alpha
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 0.7  # semi-transparent

                    # Lifespan (-1 means forever)
                    marker.lifetime.sec = 0

                    marker_array.markers.append(marker)

        return marker_array


def main(args=None):
    rclpy.init(args=args)
    node = MapObstaclePosesPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()