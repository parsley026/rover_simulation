#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from rex_interfaces.msg import VescStatus

class Nav2ParamCalculatorNode(Node):
    def __init__(self):
        super().__init__('nav2_param_calculator')
        
        # Declare parameters (These will be overwritten by your YAML file)
        self.declare_parameter('wheelbase_length', 0.900)
        self.declare_parameter('track_width', 0.654)
        self.declare_parameter('vesc_to_radians_multiplier', 1.0)
        
        self.declare_parameter('test_max_linear_vel', 1.0)
        self.declare_parameter('test_max_angular_vel', 2.0)
        self.declare_parameter('phase_timeout_sec', 10.0)
        
        self.declare_parameter('velocity_noise_deadband', 0.01)
        self.declare_parameter('ema_alpha', 0.2)
        self.declare_parameter('sustain_window_msgs', 5)
        self.declare_parameter('yaml_output_path', '/tmp/nav2_params.yaml')
        self.declare_parameter('teleop_topic', '/teleop_cmd_vel')
        
        # Internal state
        self.is_recording = False
        self.calibration_phase = 'IDLE'
        self.phase_start_time = 0.0
        self.test_velocity = 0.0
        
        # Sustained limits tracking (val, count)
        self.vx_max_cand = [0.0, 0]
        self.vx_min_cand = [0.0, 0]
        self.wz_max_cand = [0.0, 0]
        self.ax_max_cand = [0.0, 0]
        self.ax_min_cand = [0.0, 0]
        self.az_max_cand = [0.0, 0]
        
        # Verified limits
        self.limits = {
            'vx_max': 0.0,
            'vx_min': 0.0,
            'wz_max': 0.0,
            'ax_max': 0.0,
            'ax_min': 0.0,
            'az_max': 0.0,
            'min_x_velocity_threshold': float('inf'),
            'max_steer_pos_left': 0.0,
            'max_steer_pos_right': 0.0,
        }
        
        # EMA filtering
        self.ema_vx = 0.0
        self.ema_wz = 0.0
        self.last_odom_time = None
        self.current_ax = 0.0
        self.current_az = 0.0
        
        # Subscriptions & Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/kinematics/odom', self.odom_cb, 10)
        self.vesc_sub = self.create_subscription(VescStatus, '/CAN/RX/vesc_status', self.vesc_cb, 10)
        
        teleop_topic = self.get_parameter('teleop_topic').value
        self.teleop_sub = self.create_subscription(Twist, teleop_topic, self.teleop_cb, 10)
        
        # Calibration Timer (Runs at 10Hz)
        self.timer = self.create_timer(0.1, self.calibration_loop)
        
        # Services
        self.srv_start = self.create_service(Trigger, '~/start_recording', self.start_recording_cb)
        self.srv_save = self.create_service(Trigger, '~/save_yaml', self.save_yaml_cb)
        self.srv_abort = self.create_service(Trigger, '~/abort', self.abort_cb)
        
        self.get_logger().warn("Active Nav2 Param Calculator initialized. DANGER: Rover will move autonomously when started!")

    def abort_cb(self, request, response):
        self.abort_calibration("Software E-Stop triggered via service call.")
        response.success = True
        response.message = "Aborted."
        return response

    def teleop_cb(self, msg):
        # Teleop override check
        if self.is_recording and self.calibration_phase not in ['IDLE', 'DONE', 'ABORTED']:
            if msg.linear.x != 0.0 or msg.linear.y != 0.0 or msg.angular.z != 0.0:
                self.abort_calibration("Teleop override detected!")
        
        # Passthrough
        if not self.is_recording or self.calibration_phase in ['IDLE', 'DONE', 'ABORTED']:
            self.cmd_pub.publish(msg)

    def abort_calibration(self, reason):
        self.is_recording = False
        self.calibration_phase = 'ABORTED'
        msg = Twist()
        self.cmd_pub.publish(msg)
        self.get_logger().error(f"ABORTING CALIBRATION: {reason}")

    def start_recording_cb(self, request, response):
        self.is_recording = True
        self.calibration_phase = 'STATIC_FRICTION'
        self.phase_start_time = self.get_clock().now().nanoseconds / 1e9
        self.test_velocity = 0.0
        self.ema_vx = 0.0
        self.ema_wz = 0.0
        self.last_odom_time = None
        self.current_ax = 0.0
        self.current_az = 0.0
        
        self.vx_max_cand = [0.0, 0]
        self.vx_min_cand = [0.0, 0]
        self.wz_max_cand = [0.0, 0]
        self.ax_max_cand = [0.0, 0]
        self.ax_min_cand = [0.0, 0]
        self.az_max_cand = [0.0, 0]
        
        for k in self.limits:
            self.limits[k] = 0.0 if k != 'min_x_velocity_threshold' else float('inf')
            
        response.success = True
        response.message = "Started ACTIVE calibration. Stand clear!"
        self.get_logger().info(response.message)
        return response

    def save_yaml_cb(self, request, response):
        self.is_recording = False
        
        # Math for turning radius (bidirectional)
        pos_left = self.limits['max_steer_pos_left']
        pos_right = self.limits['max_steer_pos_right']
        
        mult = self.get_parameter('vesc_to_radians_multiplier').value
        L = self.get_parameter('wheelbase_length').value
        
        r_left = 0.0
        if pos_left * mult > 0.01:
            r_left = L / math.tan(pos_left * mult)
            
        r_right = 0.0
        if pos_right * mult > 0.01:
            r_right = L / math.tan(pos_right * mult)
            
        min_turning_r = max(abs(r_left), abs(r_right))
        if min_turning_r == 0.0:
            self.get_logger().warn("Max steer position was negligible. R_min defaults to 0.")
            
        min_x_thresh = self.limits['min_x_velocity_threshold']
        if min_x_thresh == float('inf'):
            min_x_thresh = 0.0

        yaml_data = {
            'controller_server': {
                'ros__parameters': {
                    'FollowPath': {
                        'max_vel_x': round(self.limits['vx_max'], 3),
                        'min_vel_x': round(self.limits['vx_min'], 3),
                        'max_vel_theta': round(self.limits['wz_max'], 3),
                        'min_x_velocity_threshold': round(min_x_thresh, 3),
                        'min_turning_radius': round(min_turning_r, 3)
                    }
                }
            },
            'velocity_smoother': {
                'ros__parameters': {
                    'max_velocity': [round(self.limits['vx_max'], 3), 0.0, round(self.limits['wz_max'], 3)],
                    'min_velocity': [round(self.limits['vx_min'], 3), 0.0, round(-self.limits['wz_max'], 3)],
                    'max_accel': [round(self.limits['ax_max'], 3), 0.0, round(self.limits['az_max'], 3)],
                    'max_decel': [round(self.limits['ax_min'], 3), 0.0, round(-self.limits['az_max'], 3)],
                }
            }
        }
        
        path = self.get_parameter('yaml_output_path').value
        try:
            with open(path, 'w') as f:
                yaml.dump(yaml_data, f, default_flow_style=False)
            response.success = True
            response.message = f"Saved YAML to {path}"
        except Exception as e:
            response.success = False
            response.message = f"Failed to save YAML: {e}"
            
        self.get_logger().info(response.message)
        return response

    def transition_phase(self, new_phase):
        self.calibration_phase = new_phase
        self.phase_start_time = self.get_clock().now().nanoseconds / 1e9
        self.test_velocity = 0.0
        self.get_logger().info(f"Transitioning to calibration phase: {new_phase}")

    def calibration_loop(self):
        if not self.is_recording or self.calibration_phase in ['IDLE', 'ABORTED']:
            return
            
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_in_phase = current_time - self.phase_start_time
        
        msg = Twist()
        
        max_lin = self.get_parameter('test_max_linear_vel').value
        max_ang = self.get_parameter('test_max_angular_vel').value
        timeout = self.get_parameter('phase_timeout_sec').value
        deadband = self.get_parameter('velocity_noise_deadband').value
        
        if self.calibration_phase == 'STATIC_FRICTION':
            self.test_velocity += 0.01
            msg.linear.x = self.test_velocity
            if abs(self.ema_vx) > deadband:
                self.limits['min_x_velocity_threshold'] = self.test_velocity
                self.get_logger().info(f"Friction overcome at: {self.test_velocity:.2f} m/s.")
                self.transition_phase('COAST_TO_STOP_1')
                
        elif self.calibration_phase == 'COAST_TO_STOP_1':
            msg.linear.x = 0.0
            if abs(self.ema_vx) < deadband and elapsed_in_phase > 1.0:
                self.transition_phase('MAX_FORWARD')
                
        elif self.calibration_phase == 'MAX_FORWARD':
            msg.linear.x = max_lin
            if (abs(self.current_ax) < 0.1 and elapsed_in_phase > 2.0) or elapsed_in_phase > timeout:
                self.transition_phase('HARD_BRAKE_TEST')
                
        elif self.calibration_phase == 'HARD_BRAKE_TEST':
            msg.linear.x = 0.0
            # Same plateau logic as MAX_FORWARD to ensure deceleration is completed
            if (abs(self.current_ax) < 0.1 and elapsed_in_phase > 2.0) or elapsed_in_phase > timeout:
                self.transition_phase('COAST_TO_STOP_2')
                
        elif self.calibration_phase == 'COAST_TO_STOP_2':
            msg.linear.x = 0.0
            if elapsed_in_phase > 2.0:
                self.transition_phase('MAX_REVERSE')
                
        elif self.calibration_phase == 'MAX_REVERSE':
            msg.linear.x = -max_lin
            if (abs(self.current_ax) < 0.1 and elapsed_in_phase > 2.0) or elapsed_in_phase > timeout:
                self.transition_phase('COAST_TO_STOP_3')
                
        elif self.calibration_phase == 'COAST_TO_STOP_3':
            msg.linear.x = 0.0
            if abs(self.ema_vx) < deadband and elapsed_in_phase > 1.0:
                self.transition_phase('MAX_STEERING_LEFT')
                
        elif self.calibration_phase == 'MAX_STEERING_LEFT':
            msg.linear.x = 1.0
            
            # Fetch track width from the parameter server
            track_width = self.get_parameter('track_width').value
            
            # Dynamically calculate max angular velocity to keep the turn center OUTSIDE the chassis
            # Set radius just 10cm outside the inner wheels
            safe_min_radius = (track_width / 2.0) + 0.1 
            dynamic_max_ang = msg.linear.x / safe_min_radius
            
            msg.angular.z = dynamic_max_ang
            
            if (abs(self.current_az) < 0.1 and elapsed_in_phase > 2.0) or elapsed_in_phase > timeout:
                self.transition_phase('MAX_STEERING_RIGHT')
                
        elif self.calibration_phase == 'MAX_STEERING_RIGHT':
            msg.linear.x = 1.0
            
            # Fetch track width from the parameter server
            track_width = self.get_parameter('track_width').value
            
            safe_min_radius = (track_width / 2.0) + 0.1 
            dynamic_max_ang = msg.linear.x / safe_min_radius
            
            msg.angular.z = -dynamic_max_ang
            
            if (abs(self.current_az) < 0.1 and elapsed_in_phase > 2.0) or elapsed_in_phase > timeout:
                self.transition_phase('DONE')
                
        elif self.calibration_phase == 'DONE':
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info("Calibration sequence complete. Call ~/save_yaml to save parameters.")
            self.calibration_phase = 'IDLE'

        self.cmd_pub.publish(msg)

    def check_peak(self, cand, val, is_max, window):
        # cand format: [highest_val_seen, consecutive_count]
        if is_max:
            # If value is at least 80% of the current peak, count it as a sustained event
            if val >= (cand[0] * 0.80):
                if val > cand[0]:
                    cand[0] = val  # Update peak, but DO NOT reset the counter
                cand[1] += 1       # Increment sustained counter
            else:
                cand[1] = 0        # Reset counter only if acceleration drops off
        else:
            # Logic for negative limits (braking and reverse acceleration)
            if val <= (cand[0] * 0.80):
                if val < cand[0]:
                    cand[0] = val
                cand[1] += 1
            else:
                cand[1] = 0
                
        return cand[1] >= window, cand[0]

    def odom_cb(self, msg):
        if not self.is_recording:
            return
            
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        
        alpha = self.get_parameter('ema_alpha').value
        window_vel = self.get_parameter('sustain_window_msgs').value
        window_accel = 2
        
        if self.last_odom_time is None:
            self.ema_vx = vx
            self.ema_wz = wz
            self.last_odom_time = current_time
            return
            
        dt = current_time - self.last_odom_time
        if dt <= 0:
            return
            
        prev_vx = self.ema_vx
        prev_wz = self.ema_wz
        self.ema_vx = alpha * vx + (1 - alpha) * prev_vx
        self.ema_wz = alpha * wz + (1 - alpha) * prev_wz
        
        self.current_ax = (self.ema_vx - prev_vx) / dt
        self.current_az = (self.ema_wz - prev_wz) / dt
        
        self.last_odom_time = current_time

        # Track Limits
        is_sustained, peak = self.check_peak(self.vx_max_cand, self.ema_vx, True, window_vel)
        if is_sustained and peak > self.limits['vx_max']: self.limits['vx_max'] = peak
        
        is_sustained, peak = self.check_peak(self.vx_min_cand, self.ema_vx, False, window_vel)
        if is_sustained and peak < self.limits['vx_min']: self.limits['vx_min'] = peak
            
        is_sustained, peak = self.check_peak(self.wz_max_cand, abs(self.ema_wz), True, window_vel)
        if is_sustained and peak > self.limits['wz_max']: self.limits['wz_max'] = peak
            
        is_sustained, peak = self.check_peak(self.ax_max_cand, self.current_ax, True, window_accel)
        if is_sustained and peak > self.limits['ax_max']: self.limits['ax_max'] = peak
            
        is_sustained, peak = self.check_peak(self.ax_min_cand, self.current_ax, False, window_accel)
        if is_sustained and peak < self.limits['ax_min']: self.limits['ax_min'] = peak
            
        is_sustained, peak = self.check_peak(self.az_max_cand, abs(self.current_az), True, window_accel)
        if is_sustained and peak > self.limits['az_max']: self.limits['az_max'] = peak

    def vesc_cb(self, msg):
        if not self.is_recording:
            return
            
        # 0x60-0x63 (96-99) are steering motors
        if msg.vesc_id in [96, 97, 98, 99]:
            pos = abs(msg.precise_pos)
            if self.calibration_phase == 'MAX_STEERING_LEFT':
                if pos > self.limits['max_steer_pos_left']:
                    self.limits['max_steer_pos_left'] = pos
            elif self.calibration_phase == 'MAX_STEERING_RIGHT':
                if pos > self.limits['max_steer_pos_right']:
                    self.limits['max_steer_pos_right'] = pos

def main(args=None):
    rclpy.init(args=args)
    node = Nav2ParamCalculatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
