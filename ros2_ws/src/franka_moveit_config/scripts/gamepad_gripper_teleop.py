#!/usr/bin/env python3

import time
from dataclasses import dataclass

from action_msgs.msg import GoalStatus
from franka_msgs.action import Grasp
from franka_msgs.action import Homing
from franka_msgs.action import Move
from franka_msgs.msg import GraspEpsilon
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


@dataclass(frozen=True)
class ButtonMap:
    enable: int
    open: int
    close: int
    homing: int
    stop: int


class GamepadGripperTeleop(Node):
    def __init__(self) -> None:
        super().__init__('gamepad_gripper_teleop_node')

        self._declare_parameters()

        self.joy_topic = str(self.get_parameter('joy_topic').value)
        self.require_enable_button = bool(
            self.get_parameter('require_enable_button').value
        )
        self.buttons = ButtonMap(
            enable=int(self.get_parameter('enable_button').value),
            open=int(self.get_parameter('button.open').value),
            close=int(self.get_parameter('button.close').value),
            homing=int(self.get_parameter('button.homing').value),
            stop=int(self.get_parameter('button.stop').value),
        )

        self.open_width = float(self.get_parameter('open.width').value)
        self.open_speed = float(self.get_parameter('open.speed').value)
        self.close_width = float(self.get_parameter('close.width').value)
        self.close_speed = float(self.get_parameter('close.speed').value)
        self.close_force = float(self.get_parameter('close.force').value)
        self.close_epsilon_inner = float(
            self.get_parameter('close.epsilon_inner').value
        )
        self.close_epsilon_outer = float(
            self.get_parameter('close.epsilon_outer').value
        )
        self.command_cooldown_sec = float(
            self.get_parameter('command_cooldown_sec').value
        )
        self.log_results = bool(self.get_parameter('log_results').value)

        gripper_ns = self._normalize_namespace(
            str(self.get_parameter('gripper_ns').value)
        )
        self.move_client = ActionClient(self, Move, f'{gripper_ns}/move')
        self.grasp_client = ActionClient(self, Grasp, f'{gripper_ns}/grasp')
        self.homing_client = ActionClient(self, Homing, f'{gripper_ns}/homing')
        self.stop_client = self.create_client(Trigger, f'{gripper_ns}/stop')

        self.previous_buttons = None
        self.active_goal_handle = None
        self.command_in_progress = False
        self.last_command_time = 0.0

        self.subscription = self.create_subscription(
            Joy,
            self.joy_topic,
            self.joy_callback,
            10,
        )

        self.get_logger().info(
            'Gamepad gripper teleop ready: hold enable button %d, '
            'open=%d, close=%d, homing=%d, stop=%d'
            % (
                self.buttons.enable,
                self.buttons.open,
                self.buttons.close,
                self.buttons.homing,
                self.buttons.stop,
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('gripper_ns', '/panda_gripper')
        self.declare_parameter('require_enable_button', True)
        self.declare_parameter('enable_button', 9)
        self.declare_parameter('button.open', 2)
        self.declare_parameter('button.close', 0)
        self.declare_parameter('button.homing', 3)
        self.declare_parameter('button.stop', 1)
        self.declare_parameter('open.width', 0.08)
        self.declare_parameter('open.speed', 0.05)
        self.declare_parameter('close.width', 0.0)
        self.declare_parameter('close.speed', 0.03)
        self.declare_parameter('close.force', 20.0)
        self.declare_parameter('close.epsilon_inner', 0.005)
        self.declare_parameter('close.epsilon_outer', 0.08)
        self.declare_parameter('command_cooldown_sec', 0.25)
        self.declare_parameter('log_results', True)

    @staticmethod
    def _normalize_namespace(namespace: str) -> str:
        namespace = namespace.strip()
        if not namespace:
            return ''
        return '/' + namespace.strip('/')

    def joy_callback(self, joy_msg: Joy) -> None:
        if self.previous_buttons is None:
            self.previous_buttons = list(joy_msg.buttons)
            return

        if self._rising_edge(joy_msg, self.buttons.stop):
            self._send_stop()
        elif self._enabled(joy_msg):
            if self._rising_edge(joy_msg, self.buttons.open):
                self._send_open()
            elif self._rising_edge(joy_msg, self.buttons.close):
                self._send_close()
            elif self._rising_edge(joy_msg, self.buttons.homing):
                self._send_homing()

        self.previous_buttons = list(joy_msg.buttons)

    def _send_open(self) -> None:
        goal = Move.Goal()
        goal.width = self.open_width
        goal.speed = self.open_speed
        self._send_action('open', self.move_client, goal)

    def _send_close(self) -> None:
        goal = Grasp.Goal()
        goal.width = self.close_width
        goal.speed = self.close_speed
        goal.force = self.close_force
        goal.epsilon = GraspEpsilon()
        goal.epsilon.inner = self.close_epsilon_inner
        goal.epsilon.outer = self.close_epsilon_outer
        self._send_action('close', self.grasp_client, goal)

    def _send_homing(self) -> None:
        self._send_action('homing', self.homing_client, Homing.Goal())

    def _send_action(self, label: str, client: ActionClient, goal) -> None:
        if not self._command_allowed(label):
            return

        if not client.server_is_ready():
            self.get_logger().warn(
                "Ignoring '%s'; gripper action server is not ready" % label
            )
            return

        self.command_in_progress = True
        self.last_command_time = time.monotonic()
        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future, command_label=label: self._goal_response_callback(
                future,
                command_label,
            )
        )
        self.get_logger().info("Sent gripper '%s' command" % label)

    def _command_allowed(self, label: str) -> bool:
        if self.command_in_progress:
            self.get_logger().warn(
                "Ignoring '%s'; gripper command already running" % label
            )
            return False

        elapsed = time.monotonic() - self.last_command_time
        if elapsed < self.command_cooldown_sec:
            return False

        return True

    def _send_stop(self) -> None:
        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()

        if not self.stop_client.service_is_ready():
            self.get_logger().warn(
                "Ignoring 'stop'; gripper stop service is not ready"
            )
            return

        self.command_in_progress = False
        self.active_goal_handle = None
        stop_future = self.stop_client.call_async(Trigger.Request())
        stop_future.add_done_callback(self._stop_response_callback)
        self.get_logger().info("Sent gripper 'stop' command")

    def _goal_response_callback(self, future, label: str) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.command_in_progress = False
            self.get_logger().error(
                "Failed to send gripper '%s' command: %s" % (label, exc)
            )
            return

        if not goal_handle.accepted:
            self.command_in_progress = False
            self.get_logger().warn("Gripper '%s' command was rejected" % label)
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future_result, command_label=label: self._result_callback(
                future_result,
                command_label,
            )
        )

    def _result_callback(self, future, label: str) -> None:
        self.command_in_progress = False
        self.active_goal_handle = None

        try:
            wrapped_result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                "Failed while waiting for gripper '%s' result: %s"
                % (label, exc)
            )
            return

        status = wrapped_result.status
        result = wrapped_result.result
        success = status == GoalStatus.STATUS_SUCCEEDED and bool(
            getattr(result, 'success', True)
        )
        error = getattr(result, 'error', '')

        if success:
            if self.log_results:
                self.get_logger().info(
                    "Gripper '%s' command completed" % label
                )
            return

        status_name = self._status_name(status)
        message = "Gripper '%s' command finished with status %s" % (
            label,
            status_name,
        )
        if error:
            message += ': ' + str(error)
        self.get_logger().warn(message)

    def _stop_response_callback(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("Failed to call gripper stop: %s" % exc)
            return

        if response.success:
            if self.log_results:
                self.get_logger().info('Gripper stop completed')
            return

        self.get_logger().warn('Gripper stop failed: %s' % response.message)

    def _enabled(self, joy_msg: Joy) -> bool:
        return (
            not self.require_enable_button
            or self._button_pressed(joy_msg, self.buttons.enable)
        )

    def _rising_edge(self, joy_msg: Joy, button_index: int) -> bool:
        if button_index < 0 or button_index >= len(joy_msg.buttons):
            return False

        previous = 0
        if button_index < len(self.previous_buttons):
            previous = self.previous_buttons[button_index]

        return joy_msg.buttons[button_index] == 1 and previous == 0

    @staticmethod
    def _button_pressed(joy_msg: Joy, button_index: int) -> bool:
        if button_index < 0 or button_index >= len(joy_msg.buttons):
            return False
        return joy_msg.buttons[button_index] == 1

    @staticmethod
    def _status_name(status: int) -> str:
        status_names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return status_names.get(status, str(status))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GamepadGripperTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
