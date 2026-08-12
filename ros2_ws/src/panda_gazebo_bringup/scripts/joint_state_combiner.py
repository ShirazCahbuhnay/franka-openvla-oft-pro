#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateCombiner(Node):
    def __init__(self):
        super().__init__('joint_state_combiner')

        self.declare_parameter('arm_joint_states_topic', '/arm/joint_states')
        self.declare_parameter(
            'finger_joint_names',
            ['panda_finger_joint1', 'panda_finger_joint2'],
        )
        self.declare_parameter('gripper_joint_states_topic', '')
        self.declare_parameter('finger_position', 0.026)
        self.declare_parameter('publish_rate_hz', 60.0)

        arm_joint_states_topic = self.get_parameter(
            'arm_joint_states_topic'
        ).get_parameter_value().string_value
        self.finger_joint_names = list(
            self.get_parameter('finger_joint_names').get_parameter_value().string_array_value
        )
        self.gripper_joint_states_topic = (
            self.get_parameter('gripper_joint_states_topic').get_parameter_value().string_value
        )
        self.finger_position = (
            self.get_parameter('finger_position').get_parameter_value().double_value
        )
        self.publish_rate_hz = (
            self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        )
        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')

        self.min_publish_period = 1.0 / self.publish_rate_hz
        self.last_publish_time = None
        self.latest_gripper_state = None

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.subscription = self.create_subscription(
            JointState,
            arm_joint_states_topic,
            self._joint_state_callback,
            10,
        )
        self.gripper_subscription = None
        if self.gripper_joint_states_topic:
            self.gripper_subscription = self.create_subscription(
                JointState,
                self.gripper_joint_states_topic,
                self._gripper_joint_state_callback,
                10,
            )
        self.get_logger().info(
            f'Bridging {arm_joint_states_topic} to /joint_states at {self.publish_rate_hz:.1f} Hz'
        )
        if self.gripper_joint_states_topic:
            self.get_logger().info(
                f'Merging gripper joint states from {self.gripper_joint_states_topic}'
            )

    @staticmethod
    def _pad(values, length):
        padded = list(values)
        if len(padded) < length:
            padded.extend([0.0] * (length - len(padded)))
        return padded[:length]

    def _joint_state_callback(self, msg: JointState):
        message_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        if self.last_publish_time is not None:
            if message_time - self.last_publish_time < self.min_publish_period:
                return
        self.last_publish_time = message_time

        output = self._build_joint_state(msg)
        self.publisher.publish(output)

    def _gripper_joint_state_callback(self, msg: JointState):
        self.latest_gripper_state = msg

    def _build_joint_state(self, arm_joint_state: JointState):
        output = JointState()
        output.header = arm_joint_state.header

        arm_names = list(arm_joint_state.name)
        output.name = arm_names.copy()

        positions = self._pad(arm_joint_state.position, len(arm_names))
        velocities = self._pad(arm_joint_state.velocity, len(arm_names))
        efforts = self._pad(arm_joint_state.effort, len(arm_names))

        gripper_positions = {}
        gripper_velocities = {}
        gripper_efforts = {}
        if self.latest_gripper_state is not None:
            gripper_names = list(self.latest_gripper_state.name)
            padded_positions = self._pad(self.latest_gripper_state.position, len(gripper_names))
            padded_velocities = self._pad(self.latest_gripper_state.velocity, len(gripper_names))
            padded_efforts = self._pad(self.latest_gripper_state.effort, len(gripper_names))
            gripper_positions = dict(zip(gripper_names, padded_positions))
            gripper_velocities = dict(zip(gripper_names, padded_velocities))
            gripper_efforts = dict(zip(gripper_names, padded_efforts))

        for finger_joint_name in self.finger_joint_names:
            position = gripper_positions.get(finger_joint_name, self.finger_position)
            velocity = gripper_velocities.get(finger_joint_name, 0.0)
            effort = gripper_efforts.get(finger_joint_name, 0.0)
            if finger_joint_name in output.name:
                joint_index = output.name.index(finger_joint_name)
                positions[joint_index] = position
                velocities[joint_index] = velocity
                efforts[joint_index] = effort
                continue
            output.name.append(finger_joint_name)
            positions.append(position)
            velocities.append(velocity)
            efforts.append(effort)

        output.position = positions
        output.velocity = velocities
        output.effort = efforts
        return output


def main(args=None):
    rclpy.init(args=args)
    node = JointStateCombiner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        if 'Unable to convert call argument to Python object' not in str(error):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
