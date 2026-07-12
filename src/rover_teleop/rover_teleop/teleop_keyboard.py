#!/usr/bin/env python3
import rclpy
import sys
import select
import termios
import tty
import threading

from rover_teleop.rover_commander import RoverCommander, AUTONOMY_CONTROL, USER_CONTROL

class TeleopKeyboard(RoverCommander):
    def __init__(self):
        """
        Initialize the keyboard teleoperation node and its control state.
        
        The node declares velocity limit and adjustment step parameters, initializes
        velocity and axis values to zero, and saves the current terminal settings for
        later restoration.
        """
        super().__init__('teleop_keyboard')
        
        self.declare_parameter('max_vel', 1.0)
        self.declare_parameter('step_size', 0.05)
        
        self.max_val = self.get_parameter('max_vel').value
        self.step = self.get_parameter('step_size').value
        
        self.vel = 0.0
        self.x_axis = 0.0
        self.y_axis = 0.0

        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        """
        Read and return one keyboard character from standard input when available.
        
        Returns:
        	str: The available character, or an empty string if no input is received within the polling interval.
        """
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_loop(self):
        """
        Process keyboard input to control rover velocity, steering, drive mode, and control mode.
        
        Keyboard commands adjust the velocity and axis values within their configured limits,
        stop the rover, select a wheel mode, or switch between manual and autonomy control.
        Manual drive commands are published while manual control is active. The terminal
        settings are restored when the loop exits.
        """
        print("Control the rover:")
        print("  w/x : Increase/Decrease velocity")
        print("  a/d : Increase/Decrease x_axis (Steering)")
        print("  q/e : Increase/Decrease y_axis (Crab)")
        print("  Space/s : Emergency Stop")
        print("  --- Driving Modes ---")
        print("  1, 2, 3 : Change Wheel Mode (Advance, Crab, Spin)")
        print("  --- Control Modes ---")
        print("  m : Enable Manual/User Control")
        print("  n : Enable Autonomy Control (Wait for /cmd_vel)")
        print("Press CTRL-C to quit.")

        try:
            while rclpy.ok():
                key = self.get_key()
                if not key:
                    continue

                if key == 'w': self.vel = min(self.vel + self.step, self.max_val)
                elif key == 'x': self.vel = max(self.vel - self.step, -self.max_val)
                elif key == 'a': self.x_axis = min(self.x_axis + self.step, self.max_val)
                elif key == 'd': self.x_axis = max(self.x_axis - self.step, -self.max_val)
                elif key == 'q': self.y_axis = min(self.y_axis + self.step, self.max_val)
                elif key == 'e': self.y_axis = max(self.y_axis - self.step, -self.max_val)
                elif key in ['0', 's', ' ']: self.vel = self.x_axis = self.y_axis = 0.0
                elif key in ['1', '2', '3']: self.drive_mode = int(key)
                elif key == 'm': self.enable_manual()
                elif key == 'n': self.enable_autonomy()
                elif key == '\x03': break

                # Only publish manual drive commands if we are actually driving manually
                if self.control_mode == USER_CONTROL:
                    self.send_velocity(self.vel, self.x_axis, self.y_axis)
                
                # Visual feedback for the terminal
                ctrl_str = "AUTO" if self.control_mode == AUTONOMY_CONTROL else "MANUAL"
                sys.stdout.write(f"\rCtrl: {ctrl_str} | Vel: {self.vel:.2f} | X: {self.x_axis:.2f} | Y: {self.y_axis:.2f} | DriveMode: {self.drive_mode}   ")
                sys.stdout.flush()

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("\nShutting down Teleop...")

def main(args=None):
    """
    Start the ROS 2 keyboard teleoperation node and manage its shutdown.
    
    Parameters:
    	args: Optional arguments passed to ROS 2 initialization.
    """
    rclpy.init(args=args)
    node = TeleopKeyboard()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_keyboard_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.send_velocity(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()