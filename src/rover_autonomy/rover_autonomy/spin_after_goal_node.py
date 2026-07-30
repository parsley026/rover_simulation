"""
spin_after_goal_node.py
Monitoruje action serwer /navigate_to_pose.
Za każdym razem gdy cel nawigacji zostanie osiągnięty (STATUS_SUCCEEDED),
automatycznie obraca rover o zadeklarowany kąt (/spin action) w celu skanowania kamerami.

Użycie: ros2 run rover_autonomy spin_after_goal_node
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatusArray, GoalStatus
from nav2_msgs.action import Spin


class SpinAfterGoalNode(Node):

    def __init__(self):
        super().__init__('spin_after_goal_node')

        self.declare_parameter('spin_angle', 2 * math.pi)   # [rad] — domyślnie 360°
        self.declare_parameter('spin_timeout', 60.0)        # [s] (dajemy więcej czasu dla wolnego obrotu)

        self.spin_angle = self.get_parameter('spin_angle').value
        self.spin_timeout = self.get_parameter('spin_timeout').value

        self._spin_client = ActionClient(self, Spin, 'spin')

        # Śledź poprzednie statusy żeby wykryć przejście → SUCCEEDED
        self._prev_statuses: dict = {}
        self._spinning = False

        # Subskrybuj status action serwera nawigacji
        self._status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._status_callback,
            10
        )

        self.get_logger().info(
            f'SpinAfterGoalNode włączony. '
            f'Po osiągnięciu celu zrobi obrót o {math.degrees(self.spin_angle):.0f}°.'
        )

    def _status_callback(self, msg: GoalStatusArray):
        """Wykrywa przejście celu do STATUS_SUCCEEDED i triggeruje spin."""
        if self._spinning:
            return

        for status_info in msg.status_list:
            goal_id = bytes(status_info.goal_info.goal_id.uuid)
            current_status = status_info.status
            prev_status = self._prev_statuses.get(goal_id, None)

            # Wykryj przejście do SUCCEEDED
            if (current_status == GoalStatus.STATUS_SUCCEEDED
                    and prev_status != GoalStatus.STATUS_SUCCEEDED):
                self.get_logger().info('Cel osiągnięty! Zaczynam skanowanie terenu 360°...')
                self._spinning = True
                self._do_spin()

            self._prev_statuses[goal_id] = current_status

        # Wyczyść stare cele
        active_ids = {
            bytes(s.goal_info.goal_id.uuid) for s in msg.status_list
        }
        self._prev_statuses = {
            k: v for k, v in self._prev_statuses.items() if k in active_ids
        }

    def _do_spin(self):
        """Wysyła cel do Action Servera /spin z Nav2"""
        if not self._spin_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Serwer /spin niedostępny! Upewnij się że Nav2 działa.')
            self._spinning = False
            return

        spin_goal = Spin.Goal()
        spin_goal.target_yaw = self.spin_angle
        spin_goal.time_allowance.sec = int(self.spin_timeout)

        send_future = self._spin_client.send_goal_async(spin_goal)
        send_future.add_done_callback(self._spin_goal_accepted_callback)

    def _spin_goal_accepted_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Komenda obrotu została odrzucona przez kontroler.')
            self._spinning = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._spin_done_callback)

    def _spin_done_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Skanowanie zakończone pełnym sukcesem!')
        else:
            self.get_logger().warn('Obrót nie powiódł się w pełni lub został przerwany.')
        self._spinning = False


def main(args=None):
    rclpy.init(args=args)
    node = SpinAfterGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
