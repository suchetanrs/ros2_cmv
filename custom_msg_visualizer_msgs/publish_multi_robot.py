#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion, TransformStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

# Assume you have a custom message named TwoRobotPath
# which is defined as follows:
# std_msgs/Header        header
# geometry_msgs/PoseArray pose_array
# nav_msgs/Path[]        path_vector
# sensor_msgs/LaserScan[] laser_scan_vector
#
# Replace 'my_robot_interfaces' with your actual package name
# that defines TwoRobotPath.
from custom_msg_visualizer_example.msg import TwoRobotPath
# TF2 imports
from tf2_ros import TransformBroadcaster

class MultiRobotFleetPublisher(Node):
    def __init__(self):
        super().__init__('multi_robot_fleet_publisher')
        
        # Publisher for the TwoRobotPath
        self.publisher_ = self.create_publisher(
            TwoRobotPath,
            'multi_robot_fleet_status',
            10
        )
        
        # Timer to publish at 1 Hz
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.tf_broadcaster_ = TransformBroadcaster(self)

        # For demonstration, define two (x, y) positions to keep them within ~10m x 10m
        self.robot_positions = [
            [2.0, 3.0],   # Robot 1
            [7.0, 4.0]    # Robot 2
        ]

    def timer_callback(self):
        msg = TwoRobotPath()

        # 1) Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # or your global reference frame

        # 2) PoseArray for all robots
        pose_array = PoseArray()
        pose_array.header = msg.header

        for x, y in self.robot_positions:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation = Quaternion(
                x=0.0, y=0.0, z=0.0, w=1.0
            )
            pose_array.poses.append(pose)

        msg.robot_pose_array = pose_array

        # 3) nav_msgs/Path[] (one Path per robot)
        def heading_to_quaternion(heading: float):
            """
            Convert a heading (yaw angle in radians) about the Z-axis
            into a quaternion for orientation.
            """
            half = heading / 2.0
            qx = 0.0
            qy = 0.0
            qz = math.sin(half)
            qw = math.cos(half)

            from geometry_msgs.msg import Quaternion
            return Quaternion(x=qx, y=qy, z=qz, w=qw)


        path_vector = []
        for i, (x, y) in enumerate(self.robot_positions):
            path = Path()
            path.header = msg.header
            
            path.poses = []
            
            # Number of waypoints and length of path in the X-direction
            num_waypoints = 50
            forward_distance = 5.0
            
            # We'll create a shallow arc that starts at (x, y) and ends at (x + 5, y).
            # For variety, the arc will bow up or down in y slightly, via a small parabola.
            # Feel free to change amplitude or formula for a different curve!
            
            for step in range(num_waypoints):
                t = step / (num_waypoints - 1)  # goes from 0.0 -> 1.0
                
                # Parametric equations:
                #   X(t) = x + forward_distance * t
                #   Y(t) = y + amplitude * t * (1 - t)   (a simple "bump" parabola)
                # Choose amplitude as you like (e.g. 1.0 or 0.5 for a gentler curve).
                amplitude = 7.0
                x_path = x + forward_distance * t
                y_path = y + amplitude * t * (1.0 - t)   # max offset in y around t=0.5
                
                # Create the PoseStamped for this waypoint
                pose_stamped = PoseStamped()
                pose_stamped.header = path.header
                pose_stamped.pose.position.x = x_path
                pose_stamped.pose.position.y = y_path
                pose_stamped.pose.position.z = 0.0
                
                # (Optional) Compute orientation so each waypoint faces along the path.
                #   dX/dt = forward_distance
                #   dY/dt = amplitude * (1 - 2t)
                dx_dt = forward_distance
                dy_dt = amplitude * (1.0 - 2.0 * t)
                heading = math.atan2(dy_dt, dx_dt)
                
                pose_stamped.pose.orientation = heading_to_quaternion(heading)
                
                path.poses.append(pose_stamped)

                if(step > num_waypoints * 1 / 3.2):
                    break
            
            path_vector.append(path)

        # Finally, assign the entire path_vector to your MultiRobotFleetStatus message
        msg.robot_paths = path_vector

        # 4) sensor_msgs/LaserScan[] (one LaserScan per robot)
        laser_scan_vector = []
        for i, (x, y) in enumerate(self.robot_positions):
            scan = LaserScan()
            scan.header.stamp = self.get_clock().now().to_msg()
            print("laser:" + str(i))
            scan.header.frame_id = f"robot{i}/base_link"
            print(scan.header.frame_id)

            # Set typical LaserScan parameters
            scan.angle_min = 0.0
            scan.angle_max = 2.0 * math.pi
            scan.angle_increment = math.pi / 360.0  # 1 degree
            scan.time_increment = 0.0
            scan.scan_time = 1.0 / 5.0  # pretend sensor publishes at 5Hz
            scan.range_min = 0.2
            scan.range_max = 10.0

            num_readings = int(
                (scan.angle_max - scan.angle_min) / scan.angle_increment
            )

            # Create synthetic ranges that look somewhat "real"
            # e.g. random distances, with some variation
            ranges = []
            for j in range(num_readings):
                # Let's vary between 2.0m and 8.0m
                if((j > 0 and j < 40) or (j > 680 and j <= 720)):
                    r = random.uniform(1.17, 1.25)
                else:
                    r = random.uniform(1.9, 2.0)
                ranges.append(r)
            
            scan.ranges = ranges
            
            # Optionally fill intensities with dummy values
            intensities = [random.uniform(0.0, 1.0) for _ in range(num_readings)]
            scan.intensities = intensities

            laser_scan_vector.append(scan)

        msg.laser_scans = laser_scan_vector

        # 6) Publish TF transforms for each robot (ADDED PART)
        #    We'll assume the orientation is the same as the Pose we assigned above.
        for i, (x, y) in enumerate(self.robot_positions):
            transform_msg = TransformStamped()

            transform_msg.header.stamp = self.get_clock().now().to_msg()
            transform_msg.header.stamp.sec += 1  # Increment seconds by 1
            transform_msg.header.frame_id = 'map'
            transform_msg.child_frame_id = f"robot{i}/base_link"

            transform_msg.transform.translation.x = x
            transform_msg.transform.translation.y = y
            transform_msg.transform.translation.z = 0.0

            # A simple quaternion: no rotation about z
            transform_msg.transform.rotation.x = 0.0
            transform_msg.transform.rotation.y = 0.0
            transform_msg.transform.rotation.z = 0.0
            transform_msg.transform.rotation.w = 1.0

            # Broadcast the transform
            self.tf_broadcaster_.sendTransform(transform_msg)

        marker_array = MarkerArray()

        for i, (x, y) in enumerate(self.robot_positions):
            # Create a Marker for the i-th robot
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'     # Publish in the same frame as TF transform
            marker.ns = f"robot_{i}_box"
            marker.id = i                      # A unique ID per robot
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Position the box 1 meter in front of the robot,
            # so if we consider "in front" to be along +X,
            # we can do (x+1, y, z=0.5) so the box is on the ground
            # with half of its height below and half above.
            marker.pose.position.x = x + 1.75
            marker.pose.position.y = y
            marker.pose.position.z = 0.5  # half of the 1m box to sit on the ground

            # No rotation
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # 1m x 1m x 1m box
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0

            # Give it a color (e.g., red)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8  # slightly transparent

            marker.lifetime.sec = 0  # 0 => marker never auto-deletes
            marker_array.markers.append(marker)
        
        msg.obstacle_marker = marker_array

        # Publish the aggregated message
        self.publisher_.publish(msg)
        self.get_logger().info('Published TwoRobotPath for 2 robots.')

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotFleetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()