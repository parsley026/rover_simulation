#!/usr/bin/env python3
import sys
import os
import yaml
import time
import rclpy
import threading
import functools
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

class TwistGrapher(Node):
    """
    Dynamically subscribes to N Twist topics based on a YAML config
    and stores their linear.x and angular.z values for live plotting.
    """
    def __init__(self):
        super().__init__('twist_grapher')
        
        # 1. Parameter setup and YAML loading[cite: 8]
        self.declare_parameter('twist_topics', [''])
        raw_topics = self.get_parameter('twist_topics').value
        self.topics = [t for t in raw_topics if t]

        # 2. Fallback logic: Load YAML if CLI params are missing[cite: 8]
        if not self.topics:
            self.get_logger().info("No CLI parameters found. Loading default YAML from package...[cite: 8]")
            try:
                package_name = 'rover_toolbox' 
                pkg_share = get_package_share_directory(package_name)
                yaml_path = os.path.join(pkg_share, 'config', 'twist_config.yaml')
                
                with open(yaml_path, 'r') as f:
                    config = yaml.safe_load(f)
                
                node_params = config.get('twist_grapher', {}).get('ros__parameters', {})
                self.topics = node_params.get('twist_topics', [])

            except PackageNotFoundError:
                self.get_logger().fatal(f"Could not find package '{package_name}'.[cite: 8]")
                sys.exit(1)
            except FileNotFoundError:
                self.get_logger().fatal(f"Default config not found at: {yaml_path}[cite: 8]")
                sys.exit(1)

        if not self.topics:
            self.get_logger().fatal("Configuration is invalid! Check your twist_config.yaml structure.[cite: 8]")
            sys.exit(1)

        self.get_logger().info(f"Comparing Twist topics: {self.topics}")

        # 3. Data structures for plotting
        self.start_time = time.time()
        self.max_history = 200  # Number of data points to keep on screen
        self.data = {
            topic: {'time': [], 'linear_x': [], 'angular_z': []}
            for topic in self.topics
        }

        # 4. Create INDEPENDENT subscribers for each topic[cite: 8]
        self.subs = []
        for topic in self.topics:
            sub = self.create_subscription(
                Twist, 
                topic, 
                functools.partial(self.twist_callback, topic_name=topic), # Pass topic name to callback[cite: 8]
                10
            )
            self.subs.append(sub)

    def twist_callback(self, msg, topic_name):
        """
        Store the incoming Twist data with a relative timestamp.
        """
        elapsed = time.time() - self.start_time
        
        # Append new data
        self.data[topic_name]['time'].append(elapsed)
        self.data[topic_name]['linear_x'].append(msg.linear.x)
        self.data[topic_name]['angular_z'].append(msg.angular.z)
        
        # Truncate lists to keep the graph moving and memory low
        if len(self.data[topic_name]['time']) > self.max_history:
            self.data[topic_name]['time'].pop(0)
            self.data[topic_name]['linear_x'].pop(0)
            self.data[topic_name]['angular_z'].pop(0)


def update_plot(frame, node, ax_lin, ax_ang):
    """
    Matplotlib animation callback to redraw the graph.
    """
    ax_lin.clear()
    ax_ang.clear()
    
    ax_lin.set_title('Linear Velocity (X)')
    ax_lin.set_ylabel('Velocity (m/s)')
    ax_lin.grid(True)
    
    ax_ang.set_title('Angular Velocity (Z)')
    ax_ang.set_ylabel('Velocity (rad/s)')
    ax_ang.set_xlabel('Time (s)')
    ax_ang.grid(True)
    
    for topic in node.topics:
        t_data = node.data[topic]['time']
        lin_data = node.data[topic]['linear_x']
        ang_data = node.data[topic]['angular_z']
        
        if t_data:
            ax_lin.plot(t_data, lin_data, label=topic, linewidth=2)
            ax_ang.plot(t_data, ang_data, label=topic, linewidth=2)
            
    ax_lin.legend(loc='upper right')
    ax_ang.legend(loc='upper right')


def main(args=None):
    rclpy.init(args=args)
    node = TwistGrapher()

    # 1. Run ROS 2 spin() in a background thread so it doesn't block the GUI
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # 2. Setup Matplotlib on the main thread
    fig, (ax_lin, ax_ang) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title('ROS 2 Twist Velocity Grapher')
    
    # 3. Start animation loop (updates every 100ms)
    ani = animation.FuncAnimation(
        fig, update_plot, fargs=(node, ax_lin, ax_ang), interval=100
    )
    
    try:
        # Blocks until the graph window is closed
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down grapher...")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()