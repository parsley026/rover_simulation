import rclpy
from rclpy.node import Node
from rex_interfaces.msg import Wheels
import math

class KinematicsValidator(Node):
    def __init__(self):
        super().__init__('kinematics_validator')
        self.sub = self.create_subscription(Wheels, '/CAN/TX/set_motor_vel', self.callback, 10)
        
        # Physical constraints to validate against (Must match your KinematicsConfig)
        self.max_angle = 1.57 # Approx 90 degrees
        self.get_logger().info("Kinematics Validator: Advanced Mode Enabled.")

    def callback(self, msg):
        # 1. Access steering and drive values using the correct message hierarchy
        # Accessing the 'turn' (steering) and 'drive' (velocity) set_values for each wheel
        steer = [
            msg.front_left.turn.set_value,
            msg.front_right.turn.set_value,
            msg.rear_left.turn.set_value,
            msg.rear_right.turn.set_value
        ]
        
        drive = [
            msg.front_left.drive.set_value,
            msg.front_right.drive.set_value,
            msg.rear_left.drive.set_value,
            msg.rear_right.drive.set_value
        ]
        
        # 2. ARC GEOMETRY CHECK (Ackermann validation)
        # Check if front wheels are close to each other while turning
        if not math.isclose(steer[0], steer[1], abs_tol=0.1): 
            pass # Valid behavior for turns
        elif any(abs(s) > 0.1 for s in steer):
            self.get_logger().warn("Warning: Wheels are parallel while turning. Possible kinematic failure.")

        # 3. CABLING PROTECTION CHECK
        for i, s in enumerate(steer):
            if abs(s) > self.max_angle:
                self.get_logger().error(f"CRITICAL: Wheel {i} exceeds cabling limit: {s:.2f} rad")

        # 4. DIRECTIONAL INTEGRITY CHECK
        for i, v in enumerate(drive):
            # Checking if the speed is negative and steering exceeds 90 degrees (flip logic)
            if v < 0 and abs(steer[i]) > (math.pi / 2.0):
                self.get_logger().warn(f"Mismatched polarity on wheel {i}. Kinematics bridge may be fighting the flip logic.")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(KinematicsValidator())
    rclpy.shutdown()