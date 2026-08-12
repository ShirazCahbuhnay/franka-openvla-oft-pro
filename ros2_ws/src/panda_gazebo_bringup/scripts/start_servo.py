#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class StartServoClient(Node):
    def __init__(self) -> None:
        super().__init__('start_servo_client')
        self.declare_parameter('service_name', '/servo_node/start_servo')
        self.declare_parameter('wait_timeout_sec', 30.0)
        self.declare_parameter('call_timeout_sec', 30.0)

        service_name = str(self.get_parameter('service_name').value)
        self.wait_timeout_sec = float(self.get_parameter('wait_timeout_sec').value)
        self.call_timeout_sec = float(self.get_parameter('call_timeout_sec').value)
        self.client = self.create_client(Trigger, service_name)

    def run(self) -> int:
        service_name = self.client.srv_name
        self.get_logger().info(
            "Waiting up to %.1fs for MoveIt Servo start service '%s'"
            % (self.wait_timeout_sec, service_name)
        )
        if not self.client.wait_for_service(timeout_sec=self.wait_timeout_sec):
            self.get_logger().error(
                "MoveIt Servo start service '%s' was not available in time"
                % service_name
            )
            return 1

        future = self.client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.call_timeout_sec)

        if not future.done():
            self.get_logger().error(
                "Timed out after %.1fs while calling '%s'"
                % (self.call_timeout_sec, service_name)
            )
            return 1

        result = future.result()
        if result is None:
            self.get_logger().error(
                "MoveIt Servo start call to '%s' returned no result"
                % service_name
            )
            return 1

        if not result.success:
            self.get_logger().error(
                "MoveIt Servo did not start successfully: %s"
                % (result.message or "unknown error")
            )
            return 1

        self.get_logger().info(
            "MoveIt Servo started successfully: %s"
            % (result.message or "ready")
        )
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StartServoClient()
    exit_code = 0
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        sys.exit(exit_code)


if __name__ == '__main__':
    main()
