import rclpy
from rclpy.node import Node
from rex_interfaces.msg import RoverControl, RoverStatus
from nav_msgs.msg import Odometry
import threading

# --- Control Modes (RoverStatus) ---
ESTOP_CONTROL = 0
USER_CONTROL = 1
MANIPULATOR_CONTROL = 2
PROBE_CONTROL = 3
AUTONOMY_CONTROL = 4

# --- Driving Modes (RoverControl) ---
ADVANCE_MODE = 1
CRAB_MODE = 2
SPIN_MODE = 3


class RoverCommander(Node):
    """
    Base ROS 2 component for controlling the Rover. 
    Handles MQTT publishers, Odometry subscribers, and background heartbeats.
    """
    def __init__(self, node_name='rover_commander'):
        """
        Initialize the rover commander node and configure its communication interfaces.
        
        Parameters:
            node_name (str): Name assigned to the ROS 2 node.
        """
        super().__init__(node_name)
        
        self.declare_parameter('odom_topic',    '/kinematic/odom')
        self.declare_parameter('control_topic', '/MQTT/RoverControl')
        self.declare_parameter('status_topic',  '/MQTT/RoverStatus')
        
        odom_topic    = self.get_parameter('odom_topic').value
        control_topic = self.get_parameter('control_topic').value
        status_topic  = self.get_parameter('status_topic').value

        self.control_pub = self.create_publisher(RoverControl, control_topic, 10)
        self.status_pub  = self.create_publisher(RoverStatus, status_topic, 10)
        
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        
        self.lock = threading.Lock()
        self.current_pose = None
        
        # State Separation: Control vs. Driving
        self.control_mode = USER_CONTROL
        self.drive_mode = ADVANCE_MODE

        self.heartbeat_timer = self.create_timer(0.1, self._publish_heartbeat)

    def _odom_callback(self, msg):
        """Update the stored rover pose from an odometry message.
        
        Parameters:
        	msg: Odometry message containing the latest rover pose.
        """
        with self.lock:
            self.current_pose = msg.pose.pose

    def _publish_heartbeat(self):
        """Publish the rover's current connection and control status."""
        status = RoverStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.communication_state = 2   # CONNECTION_CONNECTED
        status.pad_connected = True
        status.control_mode = self.control_mode  # Broadcasts Autonomy or User mode        
        self.status_pub.publish(status)

    def set_control_mode(self, mode: int):
        """
        Set the rover's control mode.
        
        Parameters:
            mode (int): Identifier of the control mode to activate.
        """
        self.control_mode = mode
        self.get_logger().info(f"Control mode changed to {mode}")

    def enable_autonomy(self):
        """
        Switch the rover to autonomy control mode.
        """
        self.set_control_mode(AUTONOMY_CONTROL)

    def enable_manual(self):
        """Switch rover into user/manual mode."""
        self.set_control_mode(USER_CONTROL)

    def send_velocity(self, vel, x_axis=0.0, y_axis=0.0, mode=None):
        """
        Send a velocity and axis command using the selected driving mode.
        
        Parameters:
            vel: Forward velocity command.
            x_axis: Lateral or rotational axis command.
            y_axis: Secondary axis command.
            mode: Driving mode to use for this command and subsequent commands.
        """
        if mode is not None:
            self.drive_mode = mode
            
        msg = RoverControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vel = float(vel)
        msg.x_axis = float(x_axis)
        msg.y_axis = float(y_axis)
        msg.mode = self.drive_mode # e.g., Advance, Crab, or Spin
        
        self.control_pub.publish(msg)

    def get_pose(self):
        """Return the latest odometry pose.
        
        Returns:
            The most recently received odometry pose, or ``None`` if no pose is available.
        """
        with self.lock:
            return self.current_pose