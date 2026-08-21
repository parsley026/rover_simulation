#!/usr/bin/env python3
import rclpy
import sys
import select
import termios
import tty
import threading

from rover_teleop.rover_commander import (
    RoverCommander, get_mode_name,
    CONTROL_MODE_DRIVE, CONTROL_MODE_ROBOTIC_ARM,
    CONTROL_MODE_DEEP_SAMPLER, CONTROL_MODE_SURFACE_SAMPLER
)

# Keys -> subsystem bitmask
_SUBSYSTEM_KEY_MAP = {
    'm': CONTROL_MODE_DRIVE,
    'r': CONTROL_MODE_ROBOTIC_ARM,
    'f': CONTROL_MODE_DEEP_SAMPLER,
    'p': CONTROL_MODE_SURFACE_SAMPLER,
}


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
        stop the rover, select a wheel mode, or switch between any control mode.
        Manual drive commands are published while DRIVE control mode is active. The terminal
        settings are restored when the loop exits.
        """
        print("Control the rover:")
        print("  w / x   : Increase / Decrease velocity")
        print("  a / d   : Increase / Decrease x_axis (Steering)")
        print("  q / e   : Increase / Decrease y_axis (Crab)")
        print("  s / ' ' : Zero all velocity axes")
        print("  --- Driving Modes (RoverControl.mode) ---")
        print("  1       : Symmetric Ackermann (SYM)")
        print("  2       : Crab")
        print("  3       : Symmetric Spin (SYM)")
        print("  4       : Handbrake")
        print("  --- Subsystems ---")
        print("  m       : DRIVE")
        print("  r       : ARM")
        print("  f       : DEEP_SAMPLER")
        print("  p       : SURF_SAMPLER")
        print("  --- Modifiers & Overrides ---")
        print("  z       : NONE")
        print("  k       : STOP")
        print("  K       : ESTOP (Shift+k)")
        print("  c       : CONFIG")
        print("  A       : Toggle Autonomy (Shift+a)")
        print("Press Ctrl-C to quit.")

        try:
            while rclpy.ok():
                key = self.get_key()
                if not key:
                    continue

                # ── Velocity axes ──────────────────────────────────────────
                if   key == 'w': self.vel    = min(self.vel    + self.step,  self.max_val)
                elif key == 'x': self.vel    = max(self.vel    - self.step, -self.max_val)
                elif key == 'a': self.x_axis = min(self.x_axis + self.step,  self.max_val)
                elif key == 'd': self.x_axis = max(self.x_axis - self.step, -self.max_val)
                elif key == 'q': self.y_axis = min(self.y_axis + self.step,  self.max_val)
                elif key == 'e': self.y_axis = max(self.y_axis - self.step, -self.max_val)
                elif key in ['s', ' ']: self.vel = self.x_axis = self.y_axis = 0.0

                # ── Drive sub-modes ────────────────────────────────────────
                elif key in ['1', '2', '3', '4']: self.drive_mode = int(key)

                # ── Control mode switching ─────────────────────────────────
                elif key in _SUBSYSTEM_KEY_MAP:
                    self.set_subsystem(_SUBSYSTEM_KEY_MAP[key])
                elif key == 'A':
                    self.toggle_autonomy()
                elif key == 'k': self.enable_stop()
                elif key == 'K': self.enable_estop()
                elif key == 'c': self.enable_config()
                elif key == 'z': self.enable_none()

                elif key == '\x03':
                    break

                # ── Publish drive command only when DRIVE bit is active ────
                if self.control_mode & CONTROL_MODE_DRIVE:
                    self.send_velocity(self.vel, self.x_axis, self.y_axis)

                # ── Status line ────────────────────────────────────────────
                mode_str = get_mode_name(self.control_mode)
                sys.stdout.write(
                    f"\rMode: {mode_str:<24} | Vel: {self.vel:+.2f} "
                    f"| X: {self.x_axis:+.2f} | Y: {self.y_axis:+.2f} "
                    f"| DrvMode: {self.drive_mode}   "
                )
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