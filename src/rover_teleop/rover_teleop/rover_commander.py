import rclpy
from rclpy.node import Node
from rex_interfaces.msg import RoverControl, RoverStatus
from nav_msgs.msg import Odometry
import threading

COMMUNICATION_STATE_CREATED  = 0
COMMUNICATION_STATE_OPENING  = 1
COMMUNICATION_STATE_OPENED   = 2
COMMUNICATION_STATE_CLOSING  = 3
COMMUNICATION_STATE_CLOSED   = 4
COMMUNICATION_STATE_FAULTED  = 5

CONTROL_MODE_NONE                     = 0
CONTROL_MODE_ESTOP                    = 1 << 0
CONTROL_MODE_STOP                     = 1 << 1
CONTROL_MODE_CONFIG                   = 1 << 2
CONTROL_MODE_DRIVE                    = 1 << 3
CONTROL_MODE_ROBOTIC_ARM              = 1 << 4
CONTROL_MODE_DEEP_SAMPLER             = 1 << 5
CONTROL_MODE_SURFACE_SAMPLER          = 1 << 6
CONTROL_MODE_DRIVE_AUTONOMY           = 1 << 7
CONTROL_MODE_ROBOTIC_ARM_AUTONOMY     = 1 << 8
CONTROL_MODE_DEEP_SAMPLER_AUTONOMY    = 1 << 9
CONTROL_MODE_SURFACE_SAMPLER_AUTONOMY = 1 << 10

SYM_ACKERMANN_MODE = 1
CRAB_MODE          = 2
SYM_SPIN_MODE      = 3

HANDBRAKE          = 4

_MODE_LABELS = [
    (CONTROL_MODE_ESTOP,                   'ESTOP'),
    (CONTROL_MODE_STOP,                    'STOP'),
    (CONTROL_MODE_CONFIG,                  'CONFIG'),
    (CONTROL_MODE_DRIVE,                   'DRIVE'),
    (CONTROL_MODE_ROBOTIC_ARM,             'ARM'),
    (CONTROL_MODE_DEEP_SAMPLER,            'DEEP_SAMPLER'),
    (CONTROL_MODE_SURFACE_SAMPLER,         'SURFACE_SAMPLER'),
    (CONTROL_MODE_DRIVE_AUTONOMY,          'DRIVE_AUTO'),
    (CONTROL_MODE_ROBOTIC_ARM_AUTONOMY,    'ARM_AUTO'),
    (CONTROL_MODE_DEEP_SAMPLER_AUTONOMY,   'DEEP_SAMPLER_AUTO'),
    (CONTROL_MODE_SURFACE_SAMPLER_AUTONOMY,'SURFACE_SAMPLER_AUTO'),
]


def get_mode_name(mode: int) -> str:
    """Return a human-readable string for a control_mode bitmask value."""
    if mode == CONTROL_MODE_NONE:
        return 'NONE'
    active = [label for flag, label in _MODE_LABELS if mode & flag]
    return '+'.join(active) if active else f'0x{mode:04X}'

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
        self.current_subsystem = CONTROL_MODE_DRIVE
        self.autonomous = False
        self.control_mode = self.current_subsystem
        self.drive_mode = SYM_ACKERMANN_MODE

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
        status.communication_state = COMMUNICATION_STATE_OPENED
        status.control_mode = self.control_mode  # Broadcasts Autonomy or User mode        
        self.status_pub.publish(status)

    def set_control_mode(self, mode: int):
        """
        Set the rover's control mode directly.
        
        Parameters:
            mode (int): Identifier of the control mode to activate.
        """
        self.control_mode = mode
        self.get_logger().info(f"Control mode changed to {mode}")

    def set_subsystem(self, subsystem: int):
        """Set the active subsystem and apply the current autonomy state."""
        self.current_subsystem = subsystem
        self._apply_subsystem_state()

    def toggle_autonomy(self):
        """Toggle autonomy state and apply it to the current subsystem."""
        self.autonomous = not self.autonomous
        self._apply_subsystem_state()
        
    def _apply_subsystem_state(self):
        """
        Calculates the correct bitmask based on subsystem and autonomy.
        The autonomy bit for any subsystem is consistently shifted by 4 bits.
        """
        if self.autonomous:
            self.set_control_mode(self.current_subsystem << 4)
        else:
            self.set_control_mode(self.current_subsystem)

    def enable_none(self):
        """Clear all control mode bits (NONE)."""
        self.set_control_mode(CONTROL_MODE_NONE)

    def enable_estop(self):
        """Emergency-stop — highest priority, overrides everything."""
        self.set_control_mode(CONTROL_MODE_ESTOP)

    def enable_stop(self):
        """Soft stop — wheels zeroed."""
        self.set_control_mode(CONTROL_MODE_STOP)

    def enable_config(self):
        """Config mode — kinematics ignores this bit."""
        self.set_control_mode(CONTROL_MODE_CONFIG)

    def enable_manual(self):
        """Switch rover into user/manual drive mode."""
        self.set_control_mode(CONTROL_MODE_DRIVE)

    def enable_robotic_arm(self):
        """Robotic arm mode — kinematics ignores this, arm subsystem handles it."""
        self.set_control_mode(CONTROL_MODE_ROBOTIC_ARM)

    def enable_deep_sampler(self):
        """Deep sampler mode — kinematics sets X-configuration."""
        self.set_control_mode(CONTROL_MODE_DEEP_SAMPLER)

    def enable_surface_sampler(self):
        """Surface sampler mode — kinematics sets X-configuration."""
        self.set_control_mode(CONTROL_MODE_SURFACE_SAMPLER)

    def enable_autonomy(self):
        """Switch rover to autonomy / MPPI drive mode."""
        self.set_control_mode(CONTROL_MODE_DRIVE_AUTONOMY)

    def enable_robotic_arm_autonomy(self):
        """Robotic arm autonomy — kinematics ignores this bit."""
        self.set_control_mode(CONTROL_MODE_ROBOTIC_ARM_AUTONOMY)

    def enable_deep_sampler_autonomy(self):
        """Deep sampler autonomy — kinematics sets X-configuration."""
        self.set_control_mode(CONTROL_MODE_DEEP_SAMPLER_AUTONOMY)

    def enable_surface_sampler_autonomy(self):
        """Surface sampler autonomy — kinematics sets X-configuration."""
        self.set_control_mode(CONTROL_MODE_SURFACE_SAMPLER_AUTONOMY)

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
        msg.mode = self.drive_mode
        
        self.control_pub.publish(msg)

    def get_pose(self):
        """Return the latest odometry pose.
        
        Returns:
            The most recently received odometry pose, or ``None`` if no pose is available.
        """
        with self.lock:
            return self.current_pose