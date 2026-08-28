#!/usr/bin/env python3

import rclpy
import json
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Empty
from std_msgs.msg import String

from nav2_msgs.action import Spin, BackUp, DriveOnHeading, Wait

class WebActionProxy(Node):
    def __init__(self):
        super().__init__('web_action_proxy')
        
        self.get_logger().info('Initializing Web Action Proxy...')

        # Publisher for live feedback to the dashboard
        self._status_pub = self.create_publisher(String, '/web/action_status', 10)

        # Subscriber for generic JSON commands from Dashboard
        self.create_subscription(String, '/web/action_command', self.command_callback, 10)

        # Action Clients
        from nav2_msgs.action import NavigateToPose, NavigateThroughPoses, FollowWaypoints, ComputePathToPose, ComputePathThroughPoses
        self._spin_client = ActionClient(self, Spin, '/navigation/spin')
        self._backup_client = ActionClient(self, BackUp, '/navigation/backup')
        self._drive_client = ActionClient(self, DriveOnHeading, '/navigation/drive_on_heading')
        self._wait_client = ActionClient(self, Wait, '/navigation/wait')
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._nav_through_client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        self._follow_waypoints_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self._compute_path_client = ActionClient(self, ComputePathToPose, '/navigation/compute_path_to_pose')
        self._compute_path_through_client = ActionClient(self, ComputePathThroughPoses, '/navigation/compute_path_through_poses')

        # ROS Services to subscribe web app requests
        self.create_service(Empty, '/web/spin_proxy', self.spin_service_callback)
        self.create_service(Empty, '/web/backup_proxy', self.backup_service_callback)
        self.create_service(Empty, '/web/drive_proxy', self.drive_service_callback)
        self.create_service(Empty, '/web/wait_proxy', self.wait_service_callback)
        
        self.get_logger().info('Ready! Listening for commands from Dashboard.')

    def publish_status(self, action_name, status, traveled=0.0, target=0.0):
        msg = String()
        msg.data = json.dumps({
            "action": action_name,
            "status": status,
            "traveled": traveled,
            "target": target
        })
        self._status_pub.publish(msg)

    def create_pose_stamped(self, x, y, yaw_deg):
        from geometry_msgs.msg import PoseStamped
        import math
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        yaw = float(yaw_deg) * math.pi / 180.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    # ================= COMMAND TOPIC =================
    def command_callback(self, msg):
        try:
            data = json.loads(msg.data)
            action = data.get("action")
            
            if action == "/navigate_to_pose":
                self.handle_navigate_to_pose(data)
            elif action == "/navigate_through_poses":
                self.handle_navigate_through_poses(data)
            elif action == "/follow_waypoints":
                self.handle_follow_waypoints(data)
            elif action == "/navigation/compute_path_to_pose":
                self.handle_compute_path_to_pose(data)
            elif action == "/navigation/compute_path_through_poses":
                self.handle_compute_path_through_poses(data)
            else:
                self.get_logger().warning(f"Unknown action command: {action}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse command: {e}")

    def handle_navigate_to_pose(self, data):
        self.get_logger().info('Received request: Triggering NavigateToPose...')
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.publish_status("Navigate to Pose", "error")
            return


        #NAVIGATE TO POSE
        from nav2_msgs.action import NavigateToPose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(data.get('x', 0.0), data.get('y', 0.0), data.get('yaw', 0.0))
        
        self.publish_status("Navigate to Pose", "starting", 0.0, -1.0)

        send_goal_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda msg: self.publish_status("Navigate to Pose", "running", msg.feedback.distance_remaining, -1.0)
        )
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, "Navigate to Pose"))

    def handle_navigate_through_poses(self, data):
        self.get_logger().info('Received request: Triggering NavigateThroughPoses...')
        if not self._nav_through_client.wait_for_server(timeout_sec=1.0):
            self.publish_status("Navigate Through Poses", "error")
            return

        #NAVIGATE THROUGH POSES
        from nav2_msgs.action import NavigateThroughPoses
        goal_msg = NavigateThroughPoses.Goal()
        waypoints = data.get('waypoints', [])
        goal_msg.poses = [self.create_pose_stamped(wp.get('x', 0.0), wp.get('y', 0.0), wp.get('yaw', 0.0)) for wp in waypoints]
        
        self.publish_status("Navigate Through Poses", "starting", 0.0, -1.0)
        send_goal_future = self._nav_through_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda msg: self.publish_status("Navigate Through Poses", "running", msg.feedback.distance_remaining, -1.0)
        )
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, "Navigate Through Poses"))

    def handle_follow_waypoints(self, data):
        self.get_logger().info('Received request: Triggering FollowWaypoints...')
        if not self._follow_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.publish_status("Follow Waypoints", "error")
            return

        #FOLLOW WAYPOINTS
        from nav2_msgs.action import FollowWaypoints
        goal_msg = FollowWaypoints.Goal()
        waypoints = data.get('waypoints', [])
        goal_msg.poses = [self.create_pose_stamped(wp.get('x', 0.0), wp.get('y', 0.0), wp.get('yaw', 0.0)) for wp in waypoints]
        
        self.publish_status("Follow Waypoints", "starting", 0.0, len(waypoints))
        send_goal_future = self._follow_waypoints_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda msg: self.publish_status("Follow Waypoints", "running", msg.feedback.current_waypoint, len(waypoints))
        )
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, "Follow Waypoints"))

    def handle_compute_path_to_pose(self, data):
        self.get_logger().info('Received request: Triggering ComputePathToPose...')
        if not self._compute_path_client.wait_for_server(timeout_sec=1.0):
            self.publish_status("Compute Path To Pose", "error")
            return

        # COMPUTE PATH TO POSE
        from nav2_msgs.action import ComputePathToPose
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self.create_pose_stamped(data.get('x', 0.0), data.get('y', 0.0), data.get('yaw', 0.0))
        
        self.publish_status("Compute Path To Pose", "starting", 0.0, -1.0)
        send_goal_future = self._compute_path_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, "Compute Path To Pose"))

    def handle_compute_path_through_poses(self, data):
        self.get_logger().info('Received request: Triggering ComputePathThroughPoses...')
        if not self._compute_path_through_client.wait_for_server(timeout_sec=1.0):
            self.publish_status("Compute Path Through Poses", "error")
            return

        # COMPUTE PATH THROUGH POSES
        from nav2_msgs.action import ComputePathThroughPoses
        goal_msg = ComputePathThroughPoses.Goal()
        waypoints = data.get('waypoints', [])
        goal_msg.goals = [self.create_pose_stamped(wp.get('x', 0.0), wp.get('y', 0.0), wp.get('yaw', 0.0)) for wp in waypoints]
        
        self.publish_status("Compute Path Through Poses", "starting", 0.0, -1.0)
        send_goal_future = self._compute_path_through_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, "Compute Path Through Poses"))

    # ================= WAIT =================
    def wait_service_callback(self, request, response):
        self.get_logger().info('Received request: Triggering Nav2 Wait...')
        if not self._wait_client.wait_for_server(timeout_sec=1.0):
            self.publish_status("Wait", "error")
            return response

        goal_msg = Wait.Goal()
        goal_msg.time.sec = 5
        goal_msg.time.nanosec = 0
        
        self.publish_status("Wait", "starting", 0.0, 5.0)

        send_goal_future = self._wait_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda msg: self.publish_status("Wait", "running", 5.0 - msg.feedback.time_left.sec, 5.0)
        )
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, "Wait"))
        return response

    # ================= SPIN =================
    def spin_service_callback(self, request, response):
        self.get_logger().info('Received request: Triggering Nav2 Spin...')
        if not self._spin_client.wait_for_server(timeout_sec=1.0):
            self.publish_status("Spin", "error")
            return response

        goal_msg = Spin.Goal()
        goal_msg.target_yaw = 6.28  # Spin ~360 degrees
        
        self.publish_status("Spin", "starting", 0.0, goal_msg.target_yaw)
        
        send_goal_future = self._spin_client.send_goal_async(
            goal_msg, 
            feedback_callback=lambda msg: self.publish_status("Spin", "running", msg.feedback.angular_distance_traveled, 6.28)
        )
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, "Spin"))
        return response

    # ================= BACKUP =================
    def backup_service_callback(self, request, response):
        self.get_logger().info('Received request: Triggering Nav2 BackUp...')
        if not self._backup_client.wait_for_server(timeout_sec=1.0):
            self.publish_status("Backup", "error")
            return response

        goal_msg = BackUp.Goal()
        goal_msg.target.x = -0.5
        goal_msg.target.y = 0.0
        goal_msg.target.z = 0.0
        goal_msg.speed = 0.15
        
        target_dist = abs(goal_msg.target.x)
        self.publish_status("Backup", "starting", 0.0, target_dist)
        
        send_goal_future = self._backup_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda msg: self.publish_status("Backup", "running", msg.feedback.distance_traveled, target_dist)
        )
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, "Backup"))
        return response

    # ================= DRIVE ON HEADING =================
    def drive_service_callback(self, request, response):
        self.get_logger().info('Received request: Triggering Nav2 DriveOnHeading...')
        if not self._drive_client.wait_for_server(timeout_sec=1.0):
            self.publish_status("Drive on Heading", "error")
            return response

        goal_msg = DriveOnHeading.Goal()
        goal_msg.target.x = 0.5
        goal_msg.target.y = 0.0
        goal_msg.target.z = 0.0
        goal_msg.speed = 0.15
        
        target_dist = abs(goal_msg.target.x)
        self.publish_status("Drive on Heading", "starting", 0.0, target_dist)

        send_goal_future = self._drive_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda msg: self.publish_status("Drive on Heading", "running", msg.feedback.distance_traveled, target_dist)
        )
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, "Drive on Heading"))
        return response

    # ================= COMMON CALLBACKS =================
    def goal_response_callback(self, future, action_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning(f'Nav2 rejected {action_name}.')
            self.publish_status(action_name, "rejected")
            return

        self.get_logger().info(f'Nav2 accepted {action_name}. Executing...')
        goal_handle.get_result_async().add_done_callback(lambda f: self.get_result_callback(f, action_name))

    def get_result_callback(self, future, action_name):
        self.get_logger().info(f'Nav2 {action_name} Completed!')
        self.publish_status(action_name, "completed")


def main(args=None):
    rclpy.init(args=args)
    node = WebActionProxy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
