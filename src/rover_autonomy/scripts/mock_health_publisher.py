#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from rtabmap_msgs.srv import ResetPose
from dataclasses import dataclass
import time
import math

@dataclass
class Config:
    """Parameter Caching (Configuration Struct Pattern)"""
    namespace: str = 'camera_00'
    pub_hz: float = 30.0
    scenario_duration_sec: float = 5.0
    default_vel_x: float = 0.5
    spike_vel_x: float = 4.5
    teleport_jump: float = 5.0

class MockHealthPublisher(Node):
    def __init__(self):
        super().__init__('mock_health_publisher')
        
        self.config = Config()
        
        # State-based variables
        self.start_time = time.time()
        self.current_scenario = "UNKNOWN"
        self.teleport_applied = False
        
        # Odometry State (SoC: separate physical state from ROS config)
        self.state_x = 0.0
        self.state_y = 0.0
        self.state_z = 0.0
        
        # ROS 2 Interfaces
        self.odom_pub = self.create_publisher(
            Odometry, f'/{self.config.namespace}/odom_raw', 10)
            
        self.reset_twist_srv = self.create_service(
            Empty, f'/{self.config.namespace}/reset_odom', self.reset_twist_callback)
            
        self.reset_pose_srv = self.create_service(
            ResetPose, '/reset_odom_to_pose', self.reset_pose_callback)
            
        self.timer = self.create_timer(
            1.0 / self.config.pub_hz, self.timer_callback)
            
        self.get_logger().info("Mock Health Publisher Started. Cycling scenarios...")

    # ==========================================
    # MAIN CALLBACK (SRP: Orchestrator)
    # ==========================================
    def timer_callback(self):
        self.update_scenario_state()
        msg = self.generate_odometry_message()
        self.odom_pub.publish(msg)

    # ==========================================
    # STATE MANAGEMENT (Edge-triggered events)
    # ==========================================
    def get_current_scenario(self) -> str:
        elapsed = time.time() - self.start_time
        cycle = elapsed % (self.config.scenario_duration_sec * 4)
        
        if cycle < self.config.scenario_duration_sec * 1:
            return "NORMAL"
        elif cycle < self.config.scenario_duration_sec * 2:
            return "ODOM_NAN"
        elif cycle < self.config.scenario_duration_sec * 3:
            return "ODOM_TELEPORT"
        else:
            return "ODOM_VELOCITY_SPIKE"

    def update_scenario_state(self):
        new_scenario = self.get_current_scenario()
            
        # State-based Throttling (Log only on transitions)
        if new_scenario != self.current_scenario:
            if new_scenario == "ODOM_TELEPORT":
                self.teleport_applied = False
            self.get_logger().info(f"--- MOCK INJECTING SCENARIO: {new_scenario} ---")
            self.current_scenario = new_scenario

    # ==========================================
    # SERVICE CALLBACKS
    # ==========================================
    def reset_twist_callback(self, request, response):
        self.get_logger().info("MOCK: Received reset_odom (twist) request. Resetting to 0,0,0.")
        # Real RTAB-Map twist reset zeroes out the local map
        self.state_x = 0.0
        self.state_y = 0.0
        self.state_z = 0.0
        return response
        
    def reset_pose_callback(self, request, response):
        self.get_logger().info(f"MOCK: Received reset_odom_to_pose request: x={request.x:.2f}, y={request.y:.2f}, z={request.z:.2f}")
        # ACTUALLY HEAL THE MOCK STATE!
        # Accept the C++ node's safe historical pose so the jump distance returns to 0
        self.state_x = request.x
        self.state_y = request.y
        self.state_z = request.z
        return response

    # ==========================================
    # MESSAGE GENERATION (SRP)
    # ==========================================
    def generate_odometry_message(self) -> Odometry:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = f'{self.config.namespace}_base_link'
        
        vel_x = self.config.default_vel_x
        msg.pose.pose.orientation.w = 1.0
        
        # Initialize normal covariance (DRY: loops over elements)
        for i in range(36):
            val = 0.01 if i % 7 == 0 else 0.0
            msg.pose.covariance[i] = val
            msg.twist.covariance[i] = val
            
        # Apply scenario faults (Const Correctness logic via overrides)
        if self.current_scenario == "ODOM_NAN":
            msg.pose.covariance[0] = float('nan')
        elif self.current_scenario == "ODOM_TELEPORT":
            if not self.teleport_applied:
                self.state_x += self.config.teleport_jump
                self.teleport_applied = True
        elif self.current_scenario == "ODOM_VELOCITY_SPIKE":
            vel_x = self.config.spike_vel_x
            
        # Integrate position purely based on velocity
        self.state_x += vel_x * (1.0 / self.config.pub_hz)
        
        msg.pose.pose.position.x = self.state_x
        msg.pose.pose.position.y = self.state_y
        msg.pose.pose.position.z = self.state_z
        msg.twist.twist.linear.x = vel_x
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = MockHealthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Graceful shutdown to fix RCLError
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
