#!/usr/bin/env python3
# Copyright (c) 2021 Franka Emika GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import threading
import time

from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import GripperCommand
from franka_msgs.action import Grasp
from franka_msgs.action import Homing
from franka_msgs.action import Move
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class FakeGripperActionServer(Node):
    def __init__(self):
        super().__init__('fake_gripper_action_server')

        self.declare_parameter('robot_ip', '')
        self.declare_parameter('joint_names', ['panda_finger_joint1', 'panda_finger_joint2'])
        self.declare_parameter('state_publish_rate', 50)
        self.declare_parameter('feedback_publish_rate', 30)
        self.declare_parameter('default_speed', 0.1)
        self.declare_parameter('default_grasp_epsilon.inner', 0.005)
        self.declare_parameter('default_grasp_epsilon.outer', 0.005)
        self.declare_parameter('max_width', 0.08)
        self.declare_parameter('visual_max_width', 0.08)
        self.declare_parameter('initial_width', 0.08)
        self.declare_parameter('trajectory_command_topic', '')

        self.joint_names = list(
            self.get_parameter('joint_names').get_parameter_value().string_array_value
        )
        if len(self.joint_names) != 2:
            raise ValueError('joint_names must contain exactly two finger joints')
        self.declare_parameter('trajectory_joint_names', list(self.joint_names))

        self.state_publish_rate = self.get_parameter(
            'state_publish_rate'
        ).get_parameter_value().integer_value
        self.feedback_publish_rate = self.get_parameter(
            'feedback_publish_rate'
        ).get_parameter_value().integer_value
        self.default_speed = self.get_parameter(
            'default_speed'
        ).get_parameter_value().double_value
        self.max_width = self.get_parameter('max_width').get_parameter_value().double_value
        self.visual_max_width = (
            self.get_parameter('visual_max_width').get_parameter_value().double_value
        )
        initial_width = self.get_parameter('initial_width').get_parameter_value().double_value
        self.trajectory_command_topic = (
            self.get_parameter('trajectory_command_topic').get_parameter_value().string_value
        )
        self.trajectory_joint_names = list(
            self.get_parameter('trajectory_joint_names').get_parameter_value().string_array_value
        )

        if self.state_publish_rate <= 0:
            raise ValueError('state_publish_rate must be greater than zero')
        if self.feedback_publish_rate <= 0:
            raise ValueError('feedback_publish_rate must be greater than zero')
        if self.max_width <= 0.0:
            raise ValueError('max_width must be greater than zero')
        if self.visual_max_width <= 0.0:
            raise ValueError('visual_max_width must be greater than zero')
        if len(self.trajectory_joint_names) not in (1, 2):
            raise ValueError('trajectory_joint_names must contain one or two finger joints')

        self._state_lock = threading.Lock()
        self._motion_lock = threading.Lock()
        self._current_width = self._action_to_visual_width(initial_width)

        self.joint_state_publisher = self.create_publisher(JointState, '~/joint_states', 10)
        self.timer = self.create_timer(1.0 / float(self.state_publish_rate), self.publish_state)
        self.trajectory_publisher = None
        if self.trajectory_command_topic:
            self.trajectory_publisher = self.create_publisher(
                JointTrajectory,
                self.trajectory_command_topic,
                10,
            )

        self.homing_server = ActionServer(
            self,
            Homing,
            '~/homing',
            execute_callback=self.execute_homing,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.move_server = ActionServer(
            self,
            Move,
            '~/move',
            execute_callback=self.execute_move,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.grasp_server = ActionServer(
            self,
            Grasp,
            '~/grasp',
            execute_callback=self.execute_grasp,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.gripper_command_server = ActionServer(
            self,
            GripperCommand,
            '~/gripper_action',
            execute_callback=self.execute_gripper_command,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.stop_service = self.create_service(
            Trigger,
            '~/stop',
            self.execute_stop,
        )

        self.publish_state()
        self.get_logger().info(
            "Fake gripper action server ready with max width %.3f m on '%s'"
            % (self.max_width, self.get_namespace() or '/')
        )

    def destroy_node(self):
        self.homing_server.destroy()
        self.move_server.destroy()
        self.grasp_server.destroy()
        self.gripper_command_server.destroy()
        super().destroy_node()

    def goal_callback(self, _goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def execute_stop(self, _request, response):
        response.success = True
        response.message = 'Fake gripper stop requested'
        return response

    def _clamp_width(self, width):
        return min(max(float(width), 0.0), self.visual_max_width)

    def _action_to_visual_width(self, width):
        clamped_width = min(max(float(width), 0.0), self.max_width)
        return clamped_width * (self.visual_max_width / self.max_width)

    def _visual_to_action_width(self, width):
        clamped_width = min(max(float(width), 0.0), self.visual_max_width)
        return clamped_width * (self.max_width / self.visual_max_width)

    def _get_width(self):
        with self._state_lock:
            return self._current_width

    def _set_width(self, width):
        with self._state_lock:
            self._current_width = self._clamp_width(width)
        self.publish_state()

    def publish_state(self):
        width = self._get_width()
        finger_position = width / 2.0

        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = self.joint_names
        joint_states.position = [finger_position, finger_position]
        joint_states.velocity = [0.0, 0.0]
        joint_states.effort = [0.0, 0.0]
        self.joint_state_publisher.publish(joint_states)

    def _validate_target_width(self, target_width):
        target_width = float(target_width)
        if target_width < 0.0 or target_width > self.max_width:
            return False, (
                'Requested width %.3f m is outside the fake gripper range [0.000, %.3f] m'
                % (target_width, self.max_width)
            )
        return True, ''

    def _publish_trajectory_command(self, target_visual_width, duration):
        if self.trajectory_publisher is None:
            return

        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.trajectory_joint_names)

        point = JointTrajectoryPoint()
        commanded_finger_position = target_visual_width / 2.0
        if len(self.trajectory_joint_names) == 1:
            point.positions = [commanded_finger_position]
        else:
            point.positions = [commanded_finger_position, commanded_finger_position]
        clamped_duration = max(float(duration), 0.05)
        sec = int(clamped_duration)
        nanosec = int((clamped_duration - sec) * 1e9)
        point.time_from_start = DurationMsg(sec=sec, nanosec=nanosec)
        trajectory.points = [point]
        self.trajectory_publisher.publish(trajectory)

    def _simulate_motion(self, goal_handle, target_width, speed, feedback_factory):
        target_visual_width = self._action_to_visual_width(target_width)
        speed = float(speed)
        if speed <= 0.0:
            speed = self.default_speed
        speed = max(speed, 1e-4)

        start_visual_width = self._get_width()
        start_width = self._visual_to_action_width(start_visual_width)
        distance = abs(target_width - start_width)
        duration = distance / speed if distance > 0.0 else 0.0
        self._publish_trajectory_command(target_visual_width, duration)

        step_period = 1.0 / float(self.feedback_publish_rate)
        steps = max(1, int(duration / step_period))

        for step_index in range(steps):
            if goal_handle.is_cancel_requested:
                return False, 'Goal canceled'

            ratio = float(step_index + 1) / float(steps)
            current_visual_width = start_visual_width + (
                (target_visual_width - start_visual_width) * ratio
            )
            self._set_width(current_visual_width)
            goal_handle.publish_feedback(feedback_factory(current_visual_width))

            if duration > 0.0:
                time.sleep(duration / float(steps))

        self._set_width(target_visual_width)
        goal_handle.publish_feedback(feedback_factory(target_visual_width))
        return True, ''

    def execute_homing(self, goal_handle):
        with self._motion_lock:
            success, error = self._simulate_motion(
                goal_handle,
                self.max_width,
                self.default_speed,
                self._homing_feedback,
            )

        result = Homing.Result()
        result.success = success
        result.error = error
        if success:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def execute_move(self, goal_handle):
        target_width = goal_handle.request.width
        valid, error = self._validate_target_width(target_width)
        result = Move.Result()
        if not valid:
            result.success = False
            result.error = error
            goal_handle.abort()
            return result

        with self._motion_lock:
            success, error = self._simulate_motion(
                goal_handle,
                target_width,
                goal_handle.request.speed,
                self._move_feedback,
            )

        result.success = success
        result.error = error
        if success:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def execute_grasp(self, goal_handle):
        target_width = goal_handle.request.width
        valid, error = self._validate_target_width(target_width)
        result = Grasp.Result()
        if not valid:
            result.success = False
            result.error = error
            goal_handle.abort()
            return result

        with self._motion_lock:
            success, error = self._simulate_motion(
                goal_handle,
                target_width,
                goal_handle.request.speed,
                self._grasp_feedback,
            )

        result.success = success
        result.error = error
        if success:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def execute_gripper_command(self, goal_handle):
        target_width = 2.0 * goal_handle.request.command.position
        valid, error = self._validate_target_width(target_width)
        result = GripperCommand.Result()
        if not valid:
            result.position = self._get_width()
            result.effort = 0.0
            result.stalled = False
            result.reached_goal = False
            goal_handle.abort()
            return result

        with self._motion_lock:
            success, error = self._simulate_motion(
                goal_handle,
                target_width,
                self.default_speed,
                self._gripper_command_feedback,
            )

        result.position = self._get_width()
        result.effort = 0.0
        result.stalled = False
        result.position = self._visual_to_action_width(result.position)
        result.reached_goal = success
        if success:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _homing_feedback(self, current_width):
        feedback = Homing.Feedback()
        feedback.current_width = self._visual_to_action_width(current_width)
        return feedback

    def _move_feedback(self, current_width):
        feedback = Move.Feedback()
        feedback.current_width = self._visual_to_action_width(current_width)
        return feedback

    def _grasp_feedback(self, current_width):
        feedback = Grasp.Feedback()
        feedback.current_width = self._visual_to_action_width(current_width)
        return feedback

    def _gripper_command_feedback(self, current_width):
        feedback = GripperCommand.Feedback()
        feedback.position = self._visual_to_action_width(current_width)
        feedback.effort = 0.0
        feedback.stalled = False
        feedback.reached_goal = False
        return feedback


def main(args=None):
    rclpy.init(args=args)
    action_server = FakeGripperActionServer()

    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
