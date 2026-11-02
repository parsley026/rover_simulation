#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from dataclasses import dataclass
import time

@dataclass
class Config:
    """Parameter Caching (Configuration Struct Pattern)"""
    namespace: str = 'camera_00'
    img_hz: float = 30.0
    info_hz: float = 15.0
    scenario_duration_sec: float = 5.0

class MockSyncPublisher(Node):
    def __init__(self):
        super().__init__('mock_sync_publisher')
        
        self.config = Config()
        
        # State-based variables
        self.start_time = time.time()
        self.current_scenario = "UNKNOWN"
        
        # ROS 2 Interfaces
        self.img_pub = self.create_publisher(
            Image, f'/{self.config.namespace}/rgb/image_raw', 10)
        self.info_pub = self.create_publisher(
            CameraInfo, f'/{self.config.namespace}/rgb/camera_info', 10)
        
        self.img_timer = self.create_timer(
            1.0 / self.config.img_hz, self.publish_image)
        self.info_timer = self.create_timer(
            1.0 / self.config.info_hz, self.publish_info)
        
        self.get_logger().info("Mock Sync Publisher Started. Cycling scenarios...")

    # ==========================================
    # STATE MANAGEMENT (Edge-triggered events)
    # ==========================================
    def get_current_scenario(self) -> str:
        elapsed = time.time() - self.start_time
        cycle = elapsed % (self.config.scenario_duration_sec * 2)
        
        if cycle < self.config.scenario_duration_sec:
            return "NORMAL"
        return "SYNC_DEGRADATION"

    def update_scenario_state(self):
        new_scenario = self.get_current_scenario()
            
        # State-based Throttling (Log only on transitions)
        if new_scenario != self.current_scenario:
            self.get_logger().info(f"--- MOCK INJECTING SCENARIO: {new_scenario} ---")
            self.current_scenario = new_scenario

    # ==========================================
    # MESSAGE PUBLISHING (SRP)
    # ==========================================
    def publish_image(self):
        # Image is the High-Frequency Trigger, always published reliably
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.config.namespace}_rgb_optical_frame'
        msg.height = 480
        msg.width = 640
        msg.encoding = 'rgb8'
        self.img_pub.publish(msg)

    def publish_info(self):
        self.update_scenario_state()
        
        # Simulate Sync Degradation (drop rate to ~1Hz)
        if self.current_scenario == "SYNC_DEGRADATION":
            if int(time.time() * 10) % 10 != 0:
                return
                
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.config.namespace}_rgb_optical_frame'
        msg.height = 480
        msg.width = 640
        self.info_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockSyncPublisher()
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
