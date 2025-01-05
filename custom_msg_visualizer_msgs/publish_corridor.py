#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time

# Messages
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# TF
from tf2_ros import TransformBroadcaster

# Your custom message (adjust package name as needed)
from custom_msg_visualizer_example.msg import CorridorData

corridor_msg = CorridorData()

class CorridorSimNode(Node):
    def __init__(self):
        super().__init__('corridor_sim_node')
        
        # Publisher for custom CorridorData message
        self.corridor_data_pub = self.create_publisher(CorridorData, 'corridor_data', 10)
        
        # TF broadcaster to publish the transform from map -> base_link
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Simulation parameters
        self.sim_duration = 100.0  # total simulation time in seconds
        self.sim_start_time = time.time()
        
        # Create a static LaserScan message (walls on left and right only)
        self.static_laser_scan = self.create_static_laserscan()
        
        # Keep track of how many times we've published
        self.publish_count = 0
        
        # Create a timer to update and publish at 1 Hz
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info("CorridorSimNode initialized. Publishing corridor data for 10 seconds...")

    def create_static_laserscan(self) -> LaserScan:
        """
        Create a static LaserScan representing corridor walls on left and right.
        This is just an example with arbitrary angles/ranges.
        """
        scan = LaserScan()
        scan.header.frame_id = 'base_link'
        
        # Example: 180 degree scan from -90 deg to +90 deg
        scan.angle_min = -math.pi / 2
        scan.angle_max =  math.pi / 2
        scan.angle_increment = math.pi / 180  # 1 degree increments
        scan.range_min = 0.2
        scan.range_max = 10.0
        
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        
        # Suppose the robot is in a corridor 2 meters wide, 
        # so left wall is 1 m to the left, right wall is 1 m to the right.
        # This is purely illustrative, not physically rigorous.
        
        # The distance to the left or right wall in this corridor scenario
        # might be about 1 meter in each direction if the robot is centered.
        # We'll just fix them at 1.0 for the entire 180-degree field.
        
        scan.ranges = [1.0] * num_readings
        
        return scan

    def timer_callback(self):
        current_time = time.time()
        elapsed = current_time - self.sim_start_time
        
        if elapsed > self.sim_duration:
            self.get_logger().info("Simulation finished. Shutting down.")
            rclpy.shutdown()
            return
        
        # Create the custom CorridorData message to publish
        
        # Fill the header with current time (ROS time)
        global corridor_msg
        now = self.get_clock().now().to_msg()
        corridor_msg.header.stamp = now
        corridor_msg.header.frame_id = 'map'
        
        # 1) ODOMETRY
        # In this simple example, let's say the robot moves along +x by 0.1 m/s
        # No rotation. So after 'elapsed' seconds, x = 0.1 * elapsed
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        
        # Positions
        odom.pose.pose.position.x = 0.5 * elapsed
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        if(self.publish_count > 5):
            odom.pose.pose.position.y = 0.5 * (elapsed - 7)
        if(self.publish_count > 10):
            odom.pose.pose.position.y = (0.5 * (10 - 7)) - (0.5 * (elapsed - 14))
        
        # No rotation, so quaternion is 0,0,0,1
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        rotation = 0
        if(self.publish_count > 5):
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            rotation = 45
            odom.pose.pose.orientation.z = math.sin(rotation / 360 * 3.14)
            odom.pose.pose.orientation.w = math.cos(rotation / 360 * 3.14)
        if(self.publish_count > 10):
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            rotation = -45
            odom.pose.pose.orientation.z = math.sin(rotation / 360 * 3.14)
            odom.pose.pose.orientation.w = math.cos(rotation / 360 * 3.14)
            
        # Fill the twist with some constant velocity in x
        odom.twist.twist.linear.x = 0.1
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0
        
        """
        Create a MarkerArray with two boxes representing corridor walls
        on the left and right side of the robot.
        
        For simplicity, we place them at +/-1m in y relative to the robot.
        We'll ignore yaw for now or assume yaw=0 in the corridor.
        """
        marker_array = MarkerArray()
        
        # Left wall marker (for demonstration)
        left_marker = Marker()
        left_marker.header.frame_id = 'map'
        left_marker.type = Marker.CUBE
        left_marker.action = Marker.ADD
        left_marker.id = 0  # unique ID within this MarkerArray
        left_marker.scale.x = 0.1  # thickness
        left_marker.scale.y = 1.0  # length
        left_marker.scale.z = 1.0  # height
        
        left_marker.color.r = 0.0
        left_marker.color.g = 1.0
        left_marker.color.b = 0.0
        left_marker.color.a = 1.0
        
        # Position to the "left" of the robot by 1m
        left_marker.pose.position.x = odom.pose.pose.position.x
        left_marker.pose.position.y = odom.pose.pose.position.y + 1.5
        left_marker.pose.position.z = 0.5  # center it half in the Z dimension
        # Orientation: no rotation
        left_marker.pose.orientation.x = 0.0
        left_marker.pose.orientation.y = 0.0
        left_marker.pose.orientation.z = math.sin((90 + rotation) / 360 * 3.14)  # sin(90°/2)
        left_marker.pose.orientation.w = math.cos((90 + rotation) / 360 * 3.14) # cos(90°/2)
        
        # Right wall marker
        right_marker = Marker()
        right_marker.header.frame_id = 'map'
        right_marker.type = Marker.CUBE
        right_marker.action = Marker.ADD
        right_marker.id = 1  # unique ID within this MarkerArray
        right_marker.scale.x = 0.1
        right_marker.scale.y = 1.0
        right_marker.scale.z = 1.0
        
        right_marker.color.r = 0.0
        right_marker.color.g = 1.0
        right_marker.color.b = 0.0
        right_marker.color.a = 1.0
        
        # Position to the "right" of the robot by 1m
        right_marker.pose.position.x = odom.pose.pose.position.x
        right_marker.pose.position.y = odom.pose.pose.position.y - 1.5
        right_marker.pose.position.z = 0.5
        right_marker.pose.orientation.x = 0.0
        right_marker.pose.orientation.y = 0.0
        right_marker.pose.orientation.z = math.sin((90 + rotation) / 360 * 3.14)  # sin(90°/2)
        right_marker.pose.orientation.w = math.cos((90 + rotation) / 360 * 3.14) # cos(90°/2)
        
        # Append both markers
        marker_array.markers.append(left_marker)
        marker_array.markers.append(right_marker)

        corridor_msg.corridor_walls.append(marker_array)
        
        # 3) POSE WITH COVARIANCE
        # We'll create a pose with covariance that grows over time.
        # Example: covariance in x/y that increases with each step.
        pose_cov = PoseWithCovarianceStamped()
        pose_cov.header.stamp = now
        pose_cov.header.frame_id = 'map'
        
        pose_cov.pose.pose.position.x = odom.pose.pose.position.x
        pose_cov.pose.pose.position.y = odom.pose.pose.position.y
        pose_cov.pose.pose.orientation = odom.pose.pose.orientation
        
        # Simple approach: diagonal grows with each publish_count
        # e.g. covariance_x = covariance_y = self.publish_count * 0.01
        # Fill just positions x,y for demonstration. Others = 0.0
        # Covariance is a 6x6 array, we fill it as needed
        cov_x = max(0.05, 0.05 * self.publish_count)
        cov_y = max(0.05, 0.05 * self.publish_count)
        
        # Index 0 -> x, 7 -> y, 35 -> yaw in the 6x6 array
        pose_cov.pose.covariance[0] = cov_x  # x
        pose_cov.pose.covariance[7] = cov_y  # y
        pose_cov.pose.covariance[35] = 0.0001 * self.publish_count  # yaw
        
        # Add it to corridor_msg
        corridor_msg.poses.append(pose_cov)
        
        # Put the odometry into the corridor_msg
        corridor_msg.odometry = odom
        
        # Publish corridor message
        self.corridor_data_pub.publish(corridor_msg)
        self.get_logger().info(f"Published CorridorData, seq={self.publish_count}, elapsed={elapsed:.2f}s")
        
        # Publish TF from map -> base_link
        self.publish_tf(odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0)
        
        # Update count
        self.publish_count += 1

    def publish_tf(self, x, y, yaw):
        """
        Publish a TransformStamped from map to base_link 
        using the given pose (x, y, yaw).
        """
        t = TransformStamped()
        
        # Use current ROS time
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        # Translation
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        
        # Yaw to quaternion
        # For no rotation, we'll just keep it 0,0,0,1
        # If you want to handle yaw, do the conversion:
        # q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CorridorSimNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()