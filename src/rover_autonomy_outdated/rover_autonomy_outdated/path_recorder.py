#!/usr/bin/env python3
"""
Wersja 1: Pure Pursuit (AKTUALNIE WŁĄCZONA)
Wersja 2: Tape Recorder (ZAKOMENTOWANA NA DOLE).
"""

import math
import collections
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Trigger



class PathRecorderNode(Node):

    def __init__(self):
        super().__init__('path_recorder')

        # Parametry
        self.declare_parameter('odom_topic',          '/kinematic/odom')
        self.declare_parameter('cmd_vel_topic',       '/cmd_vel')
        self.declare_parameter('max_path_length_m',   10.0)
        self.declare_parameter('record_interval_m',    0.10)
        self.declare_parameter('reverse_speed',         0.30)
        self.declare_parameter('goal_tolerance_m',      0.20)
        self.declare_parameter('cmd_vel_hz',            50.0)
        self.declare_parameter('recovery_timeout_s',   60.0)

        self._odom_topic  = self.get_parameter('odom_topic').value
        self._cmd_topic   = self.get_parameter('cmd_vel_topic').value
        self._max_len     = self.get_parameter('max_path_length_m').value
        self._interval    = self.get_parameter('record_interval_m').value
        self._rev_speed   = self.get_parameter('reverse_speed').value
        self._tolerance   = self.get_parameter('goal_tolerance_m').value
        self._hz          = self.get_parameter('cmd_vel_hz').value
        self._timeout     = self.get_parameter('recovery_timeout_s').value

        self._path: collections.deque = collections.deque()
        self._total_len: float = 0.0
        self._last_recorded: tuple | None = None
        self._lock = Lock()

        self._recovering: bool = False
        self._recovery_target_idx: int = 0
        self._recovery_path: list = []
        self._recovery_start_time: float = 0.0

        self._x: float = 0.0
        self._y: float = 0.0
        self._yaw: float = 0.0

        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._odom_sub = self.create_subscription(
            Odometry, self._odom_topic, self._odom_cb, best_effort_qos)

        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, 10)
        self._path_pub = self.create_publisher(Path, '/rover_recovery/recorded_path', 10)
        self._frame_id = 'odom'

        self._escape_srv = self.create_service(
            Trigger, '/rover_recovery/escape', self._escape_cb)
        self._clear_srv = self.create_service(
            Trigger, '/rover_recovery/clear', self._clear_cb)

        period = 1.0 / self._hz
        self._ctrl_timer = self.create_timer(period, self._control_loop)

        self.get_logger().info('Pure Pursuit PathRecorder (Symmetric Ackermann Tuned) ready.')

    def _odom_cb(self, msg: Odometry) -> None:
        self._frame_id = msg.header.frame_id
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)

        self._x = x
        self._y = y
        self._yaw = yaw

        if self._recovering:
            return

        with self._lock:
            if self._last_recorded is None:
                self._record_point(x, y, yaw)
                return

            lx, ly, _ = self._last_recorded
            dist = math.hypot(x - lx, y - ly)

            if dist >= self._interval:
                while self._total_len > self._max_len and len(self._path) > 1:
                    p0 = self._path[0]
                    p1 = self._path[1]
                    seg = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                    self._total_len -= seg
                    self._path.popleft()

                self._record_point(x, y, yaw)

    def _record_point(self, x: float, y: float, yaw: float) -> None:
        if self._last_recorded is not None:
            lx, ly, _ = self._last_recorded
            self._total_len += math.hypot(x - lx, y - ly)
        self._path.append((x, y, yaw))
        self._last_recorded = (x, y, yaw)
        self._publish_path()

    def _publish_path(self) -> None:
        msg = Path()
        msg.header.frame_id = self._frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for px, py, pyaw in self._path:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = px
            p.pose.position.y = py
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self._path_pub.publish(msg)

    def _escape_cb(self, _req, response: Trigger.Response) -> Trigger.Response:
        with self._lock:
            if self._recovering:
                response.success = False
                response.message = 'Recovery już w toku!'
                return response

            if len(self._path) < 2:
                response.success = False
                response.message = 'Za mało punktów w buforze.'
                return response

            self._recovery_path = list(self._path)[:-1]
            self._recovery_path.reverse()
            self._recovery_target_idx = 0
            self._recovering = True
            self._recovery_start_time = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info('[Recovery] START - Pure Pursuit dla symetrycznego Ackermanna.')
        response.success = True
        response.message = 'Recovery started'
        return response

    def _clear_cb(self, _req, response: Trigger.Response) -> Trigger.Response:
        with self._lock:
            self._path.clear()
            self._total_len = 0.0
            self._last_recorded = None
            self._publish_path()
        response.success = True
        response.message = 'Bufor trasy wyczyszczony.'
        return response

    def _control_loop(self) -> None:
        if not self._recovering:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._recovery_start_time > self._timeout:
            self._stop_recovery('TIMEOUT')
            return

        lookahead_dist = 0.5
        
        with self._lock:
            while self._recovery_target_idx < len(self._recovery_path) - 1:
                tx, ty, _ = self._recovery_path[self._recovery_target_idx]
                if math.hypot(tx - self._x, ty - self._y) < lookahead_dist:
                    self._recovery_target_idx += 1
                else:
                    break
            
            idx = self._recovery_target_idx
            path_len = len(self._recovery_path)
            target = self._recovery_path[idx]
            is_last = (idx == path_len - 1)

        tx, ty, _ = target
        dx = tx - self._x
        dy = ty - self._y
        dist = math.hypot(dx, dy)

        if is_last and dist < self._tolerance:
            self._stop_recovery('DONE')
            return

        angle_to_target_back = math.atan2(-dy, -dx)
        angle_err = self._normalize_angle(angle_to_target_back - self._yaw)

        # Sterowanie trajektorią
        # curvature (k) = 2 * sin(alpha) / L_ld
        curvature = 2.0 * math.sin(angle_err) / max(dist, 0.1)

        # Dynamiczna kontrola prędkości
        if abs(angle_err) > math.radians(45):
            linear_x = -self._rev_speed * 0.5
        else:
            linear_x = -self._rev_speed

        #Sprawdzić czy w na łaziku nie będzie trzeba dodać minusa
        #angular_z = -abs(linear_x) * curvature
        angular_z = abs(linear_x) * curvature
        
        angular_z = max(-1.0, min(1.0, angular_z))

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self._cmd_pub.publish(twist)

    def _stop_recovery(self, reason: str) -> None:
        self._recovering = False
        self._cmd_pub.publish(Twist())
        self.get_logger().info(f'[Recovery] STOP — {reason}')
        self._path.clear()
        self._total_len = 0.0
        self._last_recorded = None
        self._publish_path()

    @staticmethod
    def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()


"""
# WERSJA 2: TAPE RECORDER (ZAKOMENTOWANA)

class PathRecorderNodeTape(Node):

    def __init__(self):
        super().__init__('path_recorder')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('record_time_s', 120.0)
        self.declare_parameter('hz',            50.0)

        self._cmd_topic = self.get_parameter('cmd_vel_topic').value
        self._max_time  = self.get_parameter('record_time_s').value
        self._hz        = self.get_parameter('hz').value

        max_len = int(self._max_time * self._hz)
        self._tape = collections.deque(maxlen=max_len)
        self._lock = Lock()

        self._recovering = False
        self._latest_v = 0.0
        self._latest_w = 0.0

        self._cmd_sub = self.create_subscription(Twist, self._cmd_topic, self._cmd_cb, 10)
        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, 10)

        self._escape_srv = self.create_service(Trigger, '/rover_recovery/escape', self._escape_cb)
        self._clear_srv = self.create_service(Trigger, '/rover_recovery/clear', self._clear_cb)

        period = 1.0 / self._hz
        self._timer = self.create_timer(period, self._loop)

    def _cmd_cb(self, msg: Twist) -> None:
        if self._recovering: return
        with self._lock:
            self._latest_v = msg.linear.x
            self._latest_w = msg.angular.z

    def _escape_cb(self, _req, response: Trigger.Response) -> Trigger.Response:
        with self._lock:
            if self._recovering:
                response.success = False
                return response
            
            while len(self._tape) > 0:
                v, w = self._tape[-1]
                if abs(v) < 0.01 and abs(w) < 0.01:
                    self._tape.pop()
                else:
                    break

            if len(self._tape) == 0:
                response.success = False
                return response

            self._recovering = True
        
        response.success = True
        return response

    def _clear_cb(self, _req, response: Trigger.Response) -> Trigger.Response:
        with self._lock:
            self._tape.clear()
        response.success = True
        return response

    def _loop(self) -> None:
        with self._lock:
            if not self._recovering:
                self._tape.append((self._latest_v, self._latest_w))
                return
            if len(self._tape) == 0:
                self._stop_playback('Koniec')
                return
            v, w = self._tape.pop()

        twist = Twist()
        twist.linear.x = -v
        twist.angular.z = -w
        self._cmd_pub.publish(twist)

    def _stop_playback(self, reason: str) -> None:
        self._recovering = False
        self._cmd_pub.publish(Twist())
        self._tape.clear()
"""
