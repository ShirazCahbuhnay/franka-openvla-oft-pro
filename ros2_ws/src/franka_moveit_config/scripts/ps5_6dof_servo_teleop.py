#!/usr/bin/env python3

from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy


@dataclass(frozen=True)
class AxisMap:
    linear_x: int
    linear_y: int
    linear_z: int
    angular_roll: int
    angular_pitch: int
    angular_yaw: int


@dataclass(frozen=True)
class ScaleMap:
    linear_x: float
    linear_y: float
    linear_z: float
    angular_roll: float
    angular_pitch: float
    angular_yaw: float


class Ps5SixDofServoTeleop(Node):
    def __init__(self) -> None:
        super().__init__('ps5_6dof_servo_teleop_node')

        self.declare_parameter('frame', 'panda_link0')
        self.declare_parameter('require_enable_button', True)
        self.declare_parameter('enable_button', 9)
        self.declare_parameter('enable_turbo_button', 10)
        self.declare_parameter('rotation_mode_axis', 4)
        self.declare_parameter('rotation_mode_button', -1)
        self.declare_parameter('rotation_mode_threshold', 0.5)

        self._declare_axis_parameters()
        self._declare_scale_parameters()

        self.frame = self.get_parameter('frame').value
        self.require_enable_button = bool(self.get_parameter('require_enable_button').value)
        self.enable_button = int(self.get_parameter('enable_button').value)
        self.enable_turbo_button = int(self.get_parameter('enable_turbo_button').value)
        self.rotation_mode_axis = int(self.get_parameter('rotation_mode_axis').value)
        self.rotation_mode_button = int(self.get_parameter('rotation_mode_button').value)
        self.rotation_mode_threshold = float(self.get_parameter('rotation_mode_threshold').value)

        self.axes = AxisMap(
            linear_x=int(self.get_parameter('axis_linear.x').value),
            linear_y=int(self.get_parameter('axis_linear.y').value),
            linear_z=int(self.get_parameter('axis_linear.z').value),
            angular_roll=int(self.get_parameter('axis_angular.roll').value),
            angular_pitch=int(self.get_parameter('axis_angular.pitch').value),
            angular_yaw=int(self.get_parameter('axis_angular.yaw').value),
        )
        self.scales = ScaleMap(
            linear_x=float(self.get_parameter('scale_linear.x').value),
            linear_y=float(self.get_parameter('scale_linear.y').value),
            linear_z=float(self.get_parameter('scale_linear.z').value),
            angular_roll=float(self.get_parameter('scale_angular.roll').value),
            angular_pitch=float(self.get_parameter('scale_angular.pitch').value),
            angular_yaw=float(self.get_parameter('scale_angular.yaw').value),
        )
        self.turbo_scales = ScaleMap(
            linear_x=float(self.get_parameter('scale_linear_turbo.x').value),
            linear_y=float(self.get_parameter('scale_linear_turbo.y').value),
            linear_z=float(self.get_parameter('scale_linear_turbo.z').value),
            angular_roll=float(self.get_parameter('scale_angular_turbo.roll').value),
            angular_pitch=float(self.get_parameter('scale_angular_turbo.pitch').value),
            angular_yaw=float(self.get_parameter('scale_angular_turbo.yaw').value),
        )

        self.publisher = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.was_enabled = False

    def _declare_axis_parameters(self) -> None:
        self.declare_parameter('axis_linear.x', 1)
        self.declare_parameter('axis_linear.y', 0)
        self.declare_parameter('axis_linear.z', 3)
        self.declare_parameter('axis_angular.roll', 0)
        self.declare_parameter('axis_angular.pitch', 1)
        self.declare_parameter('axis_angular.yaw', 2)

    def _declare_scale_parameters(self) -> None:
        self.declare_parameter('scale_linear.x', 0.10)
        self.declare_parameter('scale_linear.y', 0.10)
        self.declare_parameter('scale_linear.z', -0.08)
        self.declare_parameter('scale_linear_turbo.x', 0.20)
        self.declare_parameter('scale_linear_turbo.y', 0.20)
        self.declare_parameter('scale_linear_turbo.z', -0.12)
        self.declare_parameter('scale_angular.roll', 0.30)
        self.declare_parameter('scale_angular.pitch', 0.30)
        self.declare_parameter('scale_angular.yaw', 0.30)
        self.declare_parameter('scale_angular_turbo.roll', 0.60)
        self.declare_parameter('scale_angular_turbo.pitch', 0.60)
        self.declare_parameter('scale_angular_turbo.yaw', 0.60)

    def joy_callback(self, joy_msg: Joy) -> None:
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = str(self.frame)

        is_enabled = self._enabled(joy_msg)

        if not is_enabled:
            if self.was_enabled:
                self.publisher.publish(twist_msg)
            self.was_enabled = False
            return

        self.was_enabled = True

        scales = self.turbo_scales if self._button_pressed(joy_msg, self.enable_turbo_button) else self.scales
        if self._rotation_mode_active(joy_msg):
            twist_msg.twist.angular.x = self._axis(joy_msg, self.axes.angular_roll) * scales.angular_roll
            twist_msg.twist.angular.y = self._axis(joy_msg, self.axes.angular_pitch) * scales.angular_pitch
            twist_msg.twist.angular.z = self._axis(joy_msg, self.axes.angular_yaw) * scales.angular_yaw
        else:
            twist_msg.twist.linear.x = self._axis(joy_msg, self.axes.linear_x) * scales.linear_x
            twist_msg.twist.linear.y = self._axis(joy_msg, self.axes.linear_y) * scales.linear_y
            twist_msg.twist.linear.z = self._axis(joy_msg, self.axes.linear_z) * scales.linear_z
            twist_msg.twist.angular.z = self._axis(joy_msg, self.axes.angular_yaw) * scales.angular_yaw

        self.publisher.publish(twist_msg)

    def _enabled(self, joy_msg: Joy) -> bool:
        return not self.require_enable_button or self._button_pressed(joy_msg, self.enable_button)

    def _rotation_mode_active(self, joy_msg: Joy) -> bool:
        axis_active = self._axis(joy_msg, self.rotation_mode_axis) >= self.rotation_mode_threshold
        button_active = self._button_pressed(joy_msg, self.rotation_mode_button)
        return axis_active or button_active

    @staticmethod
    def _axis(joy_msg: Joy, index: int) -> float:
        if index < 0 or index >= len(joy_msg.axes):
            return 0.0
        return float(joy_msg.axes[index])

    @staticmethod
    def _button_pressed(joy_msg: Joy, index: int) -> bool:
        if index < 0 or index >= len(joy_msg.buttons):
            return False
        return joy_msg.buttons[index] == 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ps5SixDofServoTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
