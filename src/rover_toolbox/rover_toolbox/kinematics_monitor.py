#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rex_interfaces.msg import Wheels, VescStatus

class KinematicsMonitor(Node):
    def __init__(self):
        super().__init__('kinematics_monitor')
        
        # Subscribe to Commands (TX) and Feedback (RX)
        self.create_subscription(Wheels, '/CAN/TX/set_motor_vel', self.tx_callback, 10)
        self.create_subscription(VescStatus, '/CAN/RX/vesc_status', self.rx_callback, 10)
        
        # Buffers for the latest values
        self.cmd = {
            'FL_Drive': 0.0, 'FR_Drive': 0.0, 'RL_Drive': 0.0, 'RR_Drive': 0.0,
            'FL_Steer': 0.0, 'FR_Steer': 0.0, 'RL_Steer': 0.0, 'RR_Steer': 0.0
        }
        self.fbk = {
            'FL_Drive': 0.0, 'FR_Drive': 0.0, 'RL_Drive': 0.0, 'RR_Drive': 0.0,
            'FL_Steer': 0.0, 'FR_Steer': 0.0, 'RL_Steer': 0.0, 'RR_Steer': 0.0
        }
        
        # Map VESC IDs to dictionary keys based on KinematicsNode.cpp logic
        self.vesc_map = {
            0x50: 'FL_Drive', 0x51: 'FR_Drive', 0x52: 'RL_Drive', 0x53: 'RR_Drive',
            0x60: 'FL_Steer', 0x61: 'FR_Steer', 0x62: 'RR_Steer', 0x63: 'RL_Steer' 
        }
        
        # Update the console at 2Hz
        self.create_timer(0.5, self.print_table)

    def tx_callback(self, msg):
        """Update commanded values from /CAN/TX/set_motor_vel"""
        self.cmd['FL_Drive'] = msg.front_left.drive.set_value
        self.cmd['FR_Drive'] = msg.front_right.drive.set_value
        self.cmd['RL_Drive'] = msg.rear_left.drive.set_value
        self.cmd['RR_Drive'] = msg.rear_right.drive.set_value
        
        self.cmd['FL_Steer'] = msg.front_left.turn.set_value
        self.cmd['FR_Steer'] = msg.front_right.turn.set_value
        self.cmd['RL_Steer'] = msg.rear_left.turn.set_value
        self.cmd['RR_Steer'] = msg.rear_right.turn.set_value

    def rx_callback(self, msg):
        """Update feedback values from /CAN/RX/vesc_status"""
        if msg.vesc_id in self.vesc_map:
            key = self.vesc_map[msg.vesc_id]
            # Steer feedback uses precise_pos, Drive feedback uses erpm
            if 'Steer' in key:
                self.fbk[key] = msg.precise_pos
            else:
                self.fbk[key] = msg.erpm

    def print_table(self):
        """Clear the terminal and print the comparison table"""
        # ANSI escape code to clear screen and move cursor home
        print("\033[H\033[J", end="")
        print(f"{'Motor / Joint':<15} | {'Command (TX)':<15} | {'Feedback (RX)':<15} | {'Delta':<15}")
        print("-" * 68)
        
        for key in self.cmd:
            cmd_val = self.cmd[key]
            fbk_val = self.fbk[key]
            delta = cmd_val - fbk_val
            print(f"{key:<15} | {cmd_val:>15.2f} | {fbk_val:>15.2f} | {delta:>15.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()