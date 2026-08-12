#!/usr/bin/env python3

import sys
import time

from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class StartServoClient(Node):
    def __init__(self) -> None:
        super().__init__('start_servo_client')
        self.declare_parameter('service_name', '/servo_node/start_servo')
        self.declare_parameter('wait_timeout_sec', 30.0)
        self.declare_parameter('call_timeout_sec', 30.0)
        self.declare_parameter(
            'controller_manager_list_controllers_service',
            '/controller_manager/list_controllers',
        )
        self.declare_parameter(
            'required_controllers',
            ['panda_arm_controller', 'joint_state_broadcaster'],
        )
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter(
            'required_joint_state_names',
            [
                'panda_joint1',
                'panda_joint2',
                'panda_joint3',
                'panda_joint4',
                'panda_joint5',
                'panda_joint6',
                'panda_joint7',
            ],
        )
        self.declare_parameter('required_joint_state_samples', 3)
        self.declare_parameter('joint_state_freshness_sec', 1.0)

        service_name = str(self.get_parameter('service_name').value)
        self.wait_timeout_sec = float(self.get_parameter('wait_timeout_sec').value)
        self.call_timeout_sec = float(self.get_parameter('call_timeout_sec').value)
        self.client = self.create_client(Trigger, service_name)
        list_controllers_service = str(
            self.get_parameter('controller_manager_list_controllers_service').value
        )
        self.list_controllers_client = self.create_client(
            ListControllers,
            list_controllers_service,
        )
        self.required_controllers = list(
            self.get_parameter('required_controllers').value
        )
        self.joint_states_topic = str(self.get_parameter('joint_states_topic').value)
        self.required_joint_state_names = set(
            self.get_parameter('required_joint_state_names').value
        )
        self.required_joint_state_samples = int(
            self.get_parameter('required_joint_state_samples').value
        )
        self.joint_state_freshness_sec = float(
            self.get_parameter('joint_state_freshness_sec').value
        )
        self._valid_joint_state_sample_count = 0
        self._last_joint_state_monotonic = None

    def _time_remaining(self, deadline: float) -> float:
        return max(0.0, deadline - time.monotonic())

    def _wait_for_active_controllers(self, deadline: float) -> bool:
        if not self.required_controllers:
            return True

        service_name = self.list_controllers_client.srv_name
        self.get_logger().info(
            "Waiting for active controllers via '%s': %s"
            % (service_name, ', '.join(self.required_controllers))
        )
        while self._time_remaining(deadline) > 0.0:
            if not self.list_controllers_client.wait_for_service(timeout_sec=0.5):
                continue

            future = self.list_controllers_client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=min(2.0, self._time_remaining(deadline)),
            )
            if not future.done():
                continue

            result = future.result()
            if result is None:
                self.get_logger().warn(
                    "Controller manager returned no controller list; retrying"
                )
                continue

            active_controllers = {
                controller.name
                for controller in result.controller
                if controller.state == 'active'
            }
            missing_controllers = [
                name
                for name in self.required_controllers
                if name not in active_controllers
            ]
            if not missing_controllers:
                self.get_logger().info(
                    "Required controllers are active: %s"
                    % ', '.join(self.required_controllers)
                )
                return True

            self.get_logger().info(
                "Still waiting for active controllers: %s"
                % ', '.join(missing_controllers)
            )
            time.sleep(0.2)

        self.get_logger().error(
            "Timed out waiting for active controllers: %s"
            % ', '.join(self.required_controllers)
        )
        return False

    def _joint_state_callback(self, msg: JointState) -> None:
        joint_names = set(msg.name)
        if not self.required_joint_state_names.issubset(joint_names):
            return

        self._valid_joint_state_sample_count += 1
        self._last_joint_state_monotonic = time.monotonic()

    def _wait_for_fresh_joint_states(self, deadline: float) -> bool:
        if not self.required_joint_state_names:
            return True

        self.get_logger().info(
            "Waiting for %d fresh joint state sample(s) on '%s'"
            % (self.required_joint_state_samples, self.joint_states_topic)
        )
        subscription = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self._joint_state_callback,
            10,
        )
        try:
            while self._time_remaining(deadline) > 0.0:
                rclpy.spin_once(
                    self,
                    timeout_sec=min(0.2, self._time_remaining(deadline)),
                )
                last_sample = self._last_joint_state_monotonic
                if last_sample is None:
                    continue

                if (
                    self._valid_joint_state_sample_count
                    >= self.required_joint_state_samples
                    and time.monotonic() - last_sample <= self.joint_state_freshness_sec
                ):
                    self.get_logger().info(
                        "Received fresh joint states for Panda arm joints"
                    )
                    return True
        finally:
            self.destroy_subscription(subscription)

        self.get_logger().error(
            "Timed out waiting for fresh joint states on '%s'"
            % self.joint_states_topic
        )
        return False

    def run(self) -> int:
        readiness_deadline = time.monotonic() + self.wait_timeout_sec
        if not self._wait_for_active_controllers(readiness_deadline):
            return 1
        if not self._wait_for_fresh_joint_states(readiness_deadline):
            return 1

        service_name = self.client.srv_name
        self.get_logger().info(
            "Waiting up to %.1fs for MoveIt Servo start service '%s'"
            % (self._time_remaining(readiness_deadline), service_name)
        )
        if not self.client.wait_for_service(
            timeout_sec=self._time_remaining(readiness_deadline)
        ):
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
