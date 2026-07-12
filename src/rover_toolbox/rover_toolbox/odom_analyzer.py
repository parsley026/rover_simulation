#!/usr/bin/env python3
import sys
import os
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import message_filters
import math
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

class OdomAnalyzer(Node):
    """
    Dynamically analyzes the error between N calculated odometries and 1 simulation ground truth.
    Strictly driven by a ROS 2 YAML parameter file.
    """
    def __init__(self):
        """
        Initialize the odometry analyzer, configure its topics, and set up synchronized subscriptions and per-topic error metrics.
        
        Raises:
            SystemExit: If the default package or configuration file cannot be found, or the resulting topic configuration is invalid.
        """
        super().__init__('odom_analyzer')
        self.metrics = {}

        # Declare parameters with empty defaults to force user configuration
        self.declare_parameter('ground_truth_topic', '')
        self.declare_parameter('kinematic_topics', [''])

        # Fetch values from the parameter server
        self.gt_topic = self.get_parameter('ground_truth_topic').value
        
        # Retrieve the list and filter out any empty strings
        raw_kin_topics = self.get_parameter('kinematic_topics').value
        self.kin_topics = [t for t in raw_kin_topics if t]

        # FALLBACK LOGIC: Exit immediately if the config wasn't provided or is empty
        if not self.gt_topic or not self.kin_topics:
            self.get_logger().info("No CLI parameters found. Loading default YAML from package...")
            
            try:
                package_name = 'rover_toolbox' 
                pkg_share = get_package_share_directory(package_name)
                yaml_path = os.path.join(pkg_share, 'config', 'odom_config.yaml')
                
                # --- THIS WAS THE MISSING PART ---
                with open(yaml_path, 'r') as f:
                    config = yaml.safe_load(f)
                
                # Extract parameters from the standard ROS 2 YAML structure
                node_params = config.get('odom_analyzer', {}).get('ros__parameters', {})
                self.gt_topic = node_params.get('ground_truth_topic', '')
                self.kin_topics = node_params.get('kinematic_topics', [])

            except PackageNotFoundError:
                self.get_logger().fatal(f"Could not find package '{package_name}'.")
                sys.exit(1)
            except FileNotFoundError:
                self.get_logger().fatal(f"Default config not found at: {yaml_path}")
                sys.exit(1)

        # Final safety check
        if not self.gt_topic or not self.kin_topics:
            self.get_logger().fatal("Configuration is invalid! Check your odom_config.yaml structure.")
            sys.exit(1)
        # ---------------------------------

        self.get_logger().info(f"Ground Truth: '{self.gt_topic}'")
        self.get_logger().info(f"Comparing against {len(self.kin_topics)} topics: {self.kin_topics}")

        # 1. Create a subscriber for the Ground Truth
        self.sub_gt = message_filters.Subscriber(self, Odometry, self.gt_topic)
        subs = [self.sub_gt]

        # 2. Create subscribers dynamically for every kinematic topic
        for topic in self.kin_topics:
            sub = message_filters.Subscriber(self, Odometry, topic)
            subs.append(sub)

        # 3. Synchronize ALL of them (GT + N Kinematic topics)
        self.ts = message_filters.ApproximateTimeSynchronizer(subs, queue_size=20, slop=0.05)
        self.ts.registerCallback(self.sync_callback)

        # 4. Dictionary to track metrics for each topic independently
        self.metrics = {
            topic: {'samples': 0, 'sum_sq_err_pos': 0.0, 'sum_sq_err_yaw': 0.0}
            for topic in self.kin_topics
        }

    def get_yaw_from_quaternion(self, q):
        """
        Convert a quaternion to its yaw angle about the Z axis.
        
        Parameters:
            q: A ROS 2 quaternion containing the orientation.
        
        Returns:
            The yaw angle in radians.
        """
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def sync_callback(self, *msgs):
        """
        Process synchronized ground-truth and kinematic odometry messages to update per-topic error metrics.
        
        Parameters:
            *msgs: A ground-truth odometry message followed by kinematic odometry messages in the configured topic order.
        """
        msg_gt = msgs[0]
        kin_msgs = msgs[1:]

        x_gt = msg_gt.pose.pose.position.x
        y_gt = msg_gt.pose.pose.position.y
        yaw_gt = self.get_yaw_from_quaternion(msg_gt.pose.pose.orientation)

        # Iterate through the synchronized kinematic messages
        for i, msg_kin in enumerate(kin_msgs):
            topic_name = self.kin_topics[i]
            
            x_kin = msg_kin.pose.pose.position.x
            y_kin = msg_kin.pose.pose.position.y
            yaw_kin = self.get_yaw_from_quaternion(msg_kin.pose.pose.orientation)

            # Calculate Errors
            error_x = x_kin - x_gt
            error_y = y_kin - y_gt
            error_pos = math.hypot(error_x, error_y)
            
            error_yaw = yaw_kin - yaw_gt
            error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw)) 

            # Update Metrics
            self.metrics[topic_name]['samples'] += 1
            self.metrics[topic_name]['sum_sq_err_pos'] += (error_pos ** 2)
            self.metrics[topic_name]['sum_sq_err_yaw'] += (error_yaw ** 2)

            samples = self.metrics[topic_name]['samples']

            # Print every 20 samples to avoid terminal spam
            if samples % 20 == 0:
                rmse_pos = math.sqrt(self.metrics[topic_name]['sum_sq_err_pos'] / samples)
                rmse_yaw = math.sqrt(self.metrics[topic_name]['sum_sq_err_yaw'] / samples)
                
                self.get_logger().info(
                    f"[{topic_name}] (Sample {samples})\n"
                    f"  Inst Error -> Pos: {error_pos:.4f} m | Yaw: {math.degrees(error_yaw):.2f}°\n"
                    f"  RMSE       -> Pos: {rmse_pos:.4f} m | Yaw: {math.degrees(rmse_yaw):.2f}°\n"
                )

def main(args=None):
    """
    Run the odometry analyzer node until it stops or is interrupted.
    
    Parameters:
        args: Optional arguments passed to ROS 2 initialization.
    """
    rclpy.init(args=args)
    
    try:
        node = OdomAnalyzer()
        rclpy.spin(node)
    except SystemExit:
        # Catch the intentional sys.exit(1) to avoid an ugly traceback in the terminal
        pass
    except KeyboardInterrupt:
        pass
    finally:
        # Check if node was actually successfully created and has metrics before printing
        if 'node' in locals() and hasattr(node, 'metrics') and node.metrics:
            node.get_logger().info("\n================ FINAL RMSE SUMMARY ================")
            for topic, data in node.metrics.items():
                samples = max(1, data['samples'])
                final_rmse_pos = math.sqrt(data['sum_sq_err_pos'] / samples)
                final_rmse_yaw = math.sqrt(data['sum_sq_err_yaw'] / samples)

                node.get_logger().info(
                    f"{topic} ({samples} samples):\n"
                    f"  Pos: {final_rmse_pos:.4f} m  |  Yaw: {math.degrees(final_rmse_yaw):.2f}°"
                )
            node.get_logger().info("====================================================")
            node.destroy_node()
            
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()