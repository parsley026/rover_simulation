#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from aruco_interfaces.msg import ArucoPosition

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * cy
    q.z = cr * cp * sy - sr * sp * cy
    return q

class ArucoOdomUpdater(Node):
    def __init__(self):
        super().__init__('aruco_odom_updater')
        
        self.declare_parameter('min_arucos', 2)
        self.declare_parameter('use_hard_reset', False)
        
        self.min_arucos = self.get_parameter('min_arucos').value
        self.use_hard_reset = self.get_parameter('use_hard_reset').value
        
        self.subscription = self.create_subscription(
            ArucoPosition,
            '/aruco/estimated_pose',
            self.aruco_callback,
            10
        )
        
        if self.use_hard_reset:
            self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/localization/set_pose', 10)
            self.get_logger().info(f"ArUco Updater started in HARD RESET mode (min_arucos: {self.min_arucos})")
        else:
            self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/aruco_pose', 10)
            self.get_logger().info(f"ArUco Updater started in FUSION mode (min_arucos: {self.min_arucos})")

    def aruco_callback(self, msg):
        if not msg.total_success or msg.arucos < self.min_arucos:
            return

        pose_msg = PoseWithCovarianceStamped()
        
        # 1. Timestamps and Frame ID
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            pose_msg.header.stamp = self.get_clock().now().to_msg()
        else:
            pose_msg.header = msg.header
            
        pose_msg.header.frame_id = 'map'
        
        # 2. Position
        pose_msg.pose.pose.position.x = float(msg.x)
        pose_msg.pose.pose.position.y = float(msg.y)
        pose_msg.pose.pose.position.z = float(msg.z)
        
        # 3. Orientation (Euler to Quaternion)
        # Note: aruco_node outputs degrees. Check for NaN.
        roll = math.radians(msg.x_angle) if not math.isnan(msg.x_angle) else 0.0
        pitch = math.radians(msg.y_angle) if not math.isnan(msg.y_angle) else 0.0
        yaw = math.radians(msg.z_angle) if not math.isnan(msg.z_angle) else 0.0
        
        pose_msg.pose.pose.orientation = euler_to_quaternion(roll, pitch, yaw)
        
        # 4. Covariance Matrix
        # Small diagonal values (0.05 for X/Y, 0.01 for yaw)
        cov = [0.0] * 36
        cov[0] = 0.05  # x
        cov[7] = 0.05  # y
        cov[14] = 0.05 # z
        cov[21] = 0.05 # roll
        cov[28] = 0.05 # pitch
        cov[35] = 0.01 # yaw
        
        pose_msg.pose.covariance = cov
        
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoOdomUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
