#!/usr/bin/env python3
"""ROS 2 bridge from an OpenVLA-OFT deployment server to Franka Servo."""

from __future__ import annotations

import math
import time
from typing import Any

import numpy as np
import rclpy
from cv_bridge import CvBridge
from franka_msgs.action import Move
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import String

try:
    import cv2
except ImportError:  # pragma: no cover - reported clearly at runtime
    cv2 = None

try:
    import json_numpy

    json_numpy.patch()
except ImportError:  # pragma: no cover - reported clearly at runtime
    json_numpy = None

try:
    import requests
except ImportError:  # pragma: no cover - reported clearly at runtime
    requests = None


def _rotation_matrix_to_rpy(rotation: np.ndarray) -> np.ndarray:
    sy = math.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0
    return np.array([roll, pitch, yaw], dtype=np.float32)


def _eef_xyz_rpy(robot_state_msg: FrankaRobotState) -> np.ndarray:
    transform = np.asarray(robot_state_msg.o_t_ee, dtype=np.float64).reshape(
        4,
        4,
        order="F",
    )
    xyz = transform[:3, 3].astype(np.float32)
    rpy = _rotation_matrix_to_rpy(transform[:3, :3])
    return np.concatenate([xyz, rpy]).astype(np.float32)


def _gripper_width(joint_state_msg: JointState | None) -> float:
    if joint_state_msg is None:
        return 0.0

    width = 0.0
    found = False
    for name, position in zip(joint_state_msg.name, joint_state_msg.position):
        lowered = name.lower()
        if "finger" in lowered or "gripper" in lowered:
            width += float(position)
            found = True
    return width if found else 0.0


def _stamp_to_seconds(msg: Any) -> float | None:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return None
    seconds = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return seconds if seconds > 0.0 else None


def _center_crop_square(image: np.ndarray) -> np.ndarray:
    height, width = image.shape[:2]
    crop = min(height, width)
    y0 = (height - crop) // 2
    x0 = (width - crop) // 2
    return image[y0 : y0 + crop, x0 : x0 + crop]


class OpenVlaOftInferenceNode(Node):
    """Queries OpenVLA-OFT and converts the returned action to Servo commands."""

    def __init__(self) -> None:
        super().__init__("openvla_oft_inference_node")

        self._declare_parameters()
        self._load_parameters()
        self._check_runtime_dependencies()

        self.bridge = CvBridge()
        self.latest_image: np.ndarray | None = None
        self.latest_image_stamp: float | None = None
        self.latest_robot_state: FrankaRobotState | None = None
        self.latest_robot_state_stamp: float | None = None
        self.latest_joint_state: JointState | None = None
        self.latest_joint_state_stamp: float | None = None
        self.instruction = self.initial_instruction
        self.request_inflight = False
        self.last_gripper_command_time = 0.0
        self.last_gripper_width_command: float | None = None
        self.current_twist: TwistStamped | None = None
        self.current_twist_until = 0.0
        self.sent_stop_after_current_twist = True

        self.twist_pub = self.create_publisher(TwistStamped, self.servo_topic, 10)
        self.gripper_client = ActionClient(self, Move, self.gripper_action_name)

        image_msg_type = CompressedImage if self.camera_compressed else Image
        self.create_subscription(image_msg_type, self.camera_topic, self._image_cb, 1)
        self.create_subscription(
            FrankaRobotState,
            self.robot_state_topic,
            self._robot_state_cb,
            10,
        )
        self.create_subscription(JointState, self.joint_states_topic, self._joint_state_cb, 10)

        instruction_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String,
            self.instruction_topic,
            self._instruction_cb,
            instruction_qos,
        )

        self.create_timer(self.query_period_sec, self._query_policy_timer_cb)
        self.create_timer(self.servo_publish_period_sec, self._servo_publish_timer_cb)

        mode = "DRY RUN" if self.dry_run else "LIVE COMMANDING"
        self.get_logger().info(
            "%s: querying %s, publishing arm commands to %s, gripper action %s"
            % (mode, self.server_url, self.servo_topic, self.gripper_action_name)
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("server_url", "http://127.0.0.1:8777/act")
        self.declare_parameter("request_timeout_sec", 10.0)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("camera_topic", "/vla_front_camera/image_raw")
        self.declare_parameter("camera_compressed", False)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter(
            "robot_state_topic",
            "/franka_robot_state_broadcaster/robot_state",
        )
        self.declare_parameter("instruction_topic", "/vla/language_instruction")
        self.declare_parameter("initial_instruction", "")
        self.declare_parameter("servo_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("gripper_action_name", "/panda_gripper/move")
        self.declare_parameter("command_frame", "panda_link0")
        self.declare_parameter("query_period_sec", 1.0)
        self.declare_parameter("servo_publish_period_sec", 0.034)
        self.declare_parameter("command_duration_sec", 0.20)
        self.declare_parameter("arm_action_mode", "integrated_delta")
        self.declare_parameter("gripper_action_mode", "absolute_binary")
        self.declare_parameter("max_gripper_width_m", 0.08)
        self.declare_parameter("gripper_speed_mps", 0.04)
        self.declare_parameter("gripper_open_threshold", 0.5)
        self.declare_parameter("gripper_command_epsilon_m", 0.004)
        self.declare_parameter("gripper_command_cooldown_sec", 0.50)
        self.declare_parameter("max_linear_speed_mps", 0.05)
        self.declare_parameter("max_angular_speed_rps", 0.20)
        self.declare_parameter("image_size", 224)
        self.declare_parameter("center_crop_image", True)
        self.declare_parameter("duplicate_primary_as_wrist", False)
        self.declare_parameter("max_image_age_sec", 2.0)
        self.declare_parameter("max_robot_state_age_sec", 2.0)
        self.declare_parameter("log_every_action", True)

    def _load_parameters(self) -> None:
        self.server_url = str(self.get_parameter("server_url").value)
        self.request_timeout_sec = float(self.get_parameter("request_timeout_sec").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.camera_compressed = bool(self.get_parameter("camera_compressed").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.robot_state_topic = str(self.get_parameter("robot_state_topic").value)
        self.instruction_topic = str(self.get_parameter("instruction_topic").value)
        self.initial_instruction = str(self.get_parameter("initial_instruction").value)
        self.servo_topic = str(self.get_parameter("servo_topic").value)
        self.gripper_action_name = str(self.get_parameter("gripper_action_name").value)
        self.command_frame = str(self.get_parameter("command_frame").value)
        self.query_period_sec = float(self.get_parameter("query_period_sec").value)
        self.servo_publish_period_sec = float(
            self.get_parameter("servo_publish_period_sec").value
        )
        self.command_duration_sec = float(self.get_parameter("command_duration_sec").value)
        self.arm_action_mode = str(self.get_parameter("arm_action_mode").value)
        self.gripper_action_mode = str(self.get_parameter("gripper_action_mode").value)
        self.max_gripper_width_m = float(self.get_parameter("max_gripper_width_m").value)
        self.gripper_speed_mps = float(self.get_parameter("gripper_speed_mps").value)
        self.gripper_open_threshold = float(
            self.get_parameter("gripper_open_threshold").value
        )
        self.gripper_command_epsilon_m = float(
            self.get_parameter("gripper_command_epsilon_m").value
        )
        self.gripper_command_cooldown_sec = float(
            self.get_parameter("gripper_command_cooldown_sec").value
        )
        self.max_linear_speed_mps = float(self.get_parameter("max_linear_speed_mps").value)
        self.max_angular_speed_rps = float(
            self.get_parameter("max_angular_speed_rps").value
        )
        self.image_size = int(self.get_parameter("image_size").value)
        self.center_crop_image = bool(self.get_parameter("center_crop_image").value)
        self.duplicate_primary_as_wrist = bool(
            self.get_parameter("duplicate_primary_as_wrist").value
        )
        self.max_image_age_sec = float(self.get_parameter("max_image_age_sec").value)
        self.max_robot_state_age_sec = float(
            self.get_parameter("max_robot_state_age_sec").value
        )
        self.log_every_action = bool(self.get_parameter("log_every_action").value)

        if self.query_period_sec <= 0.0:
            raise ValueError("query_period_sec must be positive")
        if self.servo_publish_period_sec <= 0.0:
            raise ValueError("servo_publish_period_sec must be positive")
        if self.command_duration_sec <= 0.0:
            raise ValueError("command_duration_sec must be positive")
        if self.arm_action_mode not in ("integrated_delta", "raw_twist"):
            raise ValueError("arm_action_mode must be 'integrated_delta' or 'raw_twist'")
        if self.gripper_action_mode not in ("absolute_binary", "absolute_width", "delta"):
            raise ValueError(
                "gripper_action_mode must be 'absolute_binary', 'absolute_width', or 'delta'"
            )

    def _check_runtime_dependencies(self) -> None:
        missing = []
        if cv2 is None:
            missing.append("opencv-python")
        if json_numpy is None:
            missing.append("json-numpy")
        if requests is None:
            missing.append("requests")
        if missing:
            raise RuntimeError(
                "Missing Python package(s): %s. Install them in the ROS Python "
                "environment with: python3 -m pip install %s"
                % (", ".join(missing), " ".join(missing))
            )

    def _image_cb(self, msg: CompressedImage | Image) -> None:
        try:
            if self.camera_compressed:
                bgr = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
                image = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            else:
                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn("Could not decode camera image: %s" % exc)
            return

        if self.center_crop_image:
            image = _center_crop_square(image)
        if image.shape[:2] != (self.image_size, self.image_size):
            image = cv2.resize(
                image,
                (self.image_size, self.image_size),
                interpolation=cv2.INTER_AREA,
            )

        self.latest_image = np.ascontiguousarray(image.astype(np.uint8))
        self.latest_image_stamp = _stamp_to_seconds(msg)

    def _robot_state_cb(self, msg: FrankaRobotState) -> None:
        self.latest_robot_state = msg
        self.latest_robot_state_stamp = _stamp_to_seconds(msg)

    def _joint_state_cb(self, msg: JointState) -> None:
        self.latest_joint_state = msg
        self.latest_joint_state_stamp = _stamp_to_seconds(msg)

    def _instruction_cb(self, msg: String) -> None:
        self.instruction = msg.data.strip()
        self.get_logger().info("Using instruction: '%s'" % self.instruction)

    def _query_policy_timer_cb(self) -> None:
        if self.request_inflight:
            return
        if not self._has_ready_observation():
            return

        payload = self._make_payload()
        self.request_inflight = True
        try:
            start_time = time.monotonic()
            action_chunk = self._request_action(payload)
            latency = time.monotonic() - start_time
            self.get_logger().info(f"Inference latency: {latency:.4f} seconds")
            
            if action_chunk.size == 0:
                self.get_logger().warn("OpenVLA-OFT server returned an empty action")
                return
            action = action_chunk[0]
            self._handle_action(action)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn("OpenVLA-OFT request failed: %s" % exc)
        finally:
            self.request_inflight = False

    def _has_ready_observation(self) -> bool:
        if self.latest_image is None:
            self.get_logger().warn("Waiting for camera image on %s" % self.camera_topic)
            return False
        if self.latest_robot_state is None:
            self.get_logger().warn("Waiting for robot state on %s" % self.robot_state_topic)
            return False
        if self.latest_joint_state is None:
            self.get_logger().warn("Waiting for joint states on %s" % self.joint_states_topic)
            return False
        if not self.instruction:
            self.get_logger().warn(
                "Waiting for instruction on %s or parameter initial_instruction"
                % self.instruction_topic
            )
            return False

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (
            self.latest_image_stamp is not None
            and now_sec - self.latest_image_stamp > self.max_image_age_sec
        ):
            self.get_logger().warn("Latest camera image is stale")
            return False
        if (
            self.latest_robot_state_stamp is not None
            and now_sec - self.latest_robot_state_stamp > self.max_robot_state_age_sec
        ):
            self.get_logger().warn("Latest robot state is stale")
            return False
        return True

    def _make_payload(self) -> dict[str, Any]:
        assert self.latest_image is not None
        assert self.latest_robot_state is not None

        gripper_width = _gripper_width(self.latest_joint_state)
        state = np.concatenate(
            [
                _eef_xyz_rpy(self.latest_robot_state),
                np.array([0.0, gripper_width], dtype=np.float32),
            ]
        ).astype(np.float32)
        payload: dict[str, Any] = {
            "full_image": self.latest_image,
            "state": state,
            "instruction": self.instruction,
        }
        if self.duplicate_primary_as_wrist:
            payload["wrist_image"] = self.latest_image
        return payload

    def _request_action(self, payload: dict[str, Any]) -> np.ndarray:
        response = requests.post(
            self.server_url,
            json={"encoded": json_numpy.dumps(payload)},
            timeout=self.request_timeout_sec,
        )
        response.raise_for_status()
        result = response.json()
        if isinstance(result, str):
            result = json_numpy.loads(result)
        action_chunk = np.asarray(result, dtype=np.float32)
        if action_chunk.ndim == 1:
            action_chunk = action_chunk.reshape(1, -1)
        if action_chunk.shape[1] < 7:
            raise ValueError("Expected action dimension >= 7, got %s" % (action_chunk.shape,))
        return action_chunk[:, :7]

    def _handle_action(self, action: np.ndarray) -> None:
        twist = self._action_to_twist(action)
        gripper_width = self._action_to_gripper_width(float(action[-1]))

        if self.log_every_action:
            self.get_logger().info(
                "action=%s twist=[%.4f %.4f %.4f %.4f %.4f %.4f] gripper_width=%.3f dry_run=%s"
                % (
                    np.array2string(action, precision=4, suppress_small=True),
                    twist.twist.linear.x,
                    twist.twist.linear.y,
                    twist.twist.linear.z,
                    twist.twist.angular.x,
                    twist.twist.angular.y,
                    twist.twist.angular.z,
                    gripper_width,
                    self.dry_run,
                )
            )

        if self.dry_run:
            return

        self.current_twist = twist
        self.current_twist_until = time.monotonic() + self.command_duration_sec
        self.sent_stop_after_current_twist = False
        self._maybe_send_gripper_command(gripper_width)

    def _action_to_twist(self, action: np.ndarray) -> TwistStamped:
        arm = np.asarray(action[:6], dtype=np.float64)
        if self.arm_action_mode == "integrated_delta":
            arm = arm / self.command_duration_sec

        linear = np.clip(arm[:3], -self.max_linear_speed_mps, self.max_linear_speed_mps)
        angular = np.clip(arm[3:6], -self.max_angular_speed_rps, self.max_angular_speed_rps)

        twist = TwistStamped()
        twist.header.frame_id = self.command_frame
        twist.twist.linear.x = float(linear[0])
        twist.twist.linear.y = float(linear[1])
        twist.twist.linear.z = float(linear[2])
        twist.twist.angular.x = float(angular[0])
        twist.twist.angular.y = float(angular[1])
        twist.twist.angular.z = float(angular[2])
        return twist

    def _action_to_gripper_width(self, gripper_action: float) -> float:
        current_width = _gripper_width(self.latest_joint_state)
        if self.gripper_action_mode == "absolute_binary":
            open_command = gripper_action >= self.gripper_open_threshold
            width = self.max_gripper_width_m if open_command else 0.0
        elif self.gripper_action_mode == "absolute_width":
            width = np.clip(gripper_action, 0.0, 1.0) * self.max_gripper_width_m
        else:
            width = current_width + gripper_action
        return float(np.clip(width, 0.0, self.max_gripper_width_m))

    def _servo_publish_timer_cb(self) -> None:
        if self.dry_run:
            return

        now = time.monotonic()
        if self.current_twist is not None and now <= self.current_twist_until:
            twist = self.current_twist
            twist.header.stamp = self.get_clock().now().to_msg()
            self.twist_pub.publish(twist)
            return

        if not self.sent_stop_after_current_twist:
            stop = TwistStamped()
            stop.header.stamp = self.get_clock().now().to_msg()
            stop.header.frame_id = self.command_frame
            self.twist_pub.publish(stop)
            self.sent_stop_after_current_twist = True
            self.current_twist = None

    def _maybe_send_gripper_command(self, width: float) -> None:
        now = time.monotonic()
        if now - self.last_gripper_command_time < self.gripper_command_cooldown_sec:
            return
        if (
            self.last_gripper_width_command is not None
            and abs(width - self.last_gripper_width_command)
            < self.gripper_command_epsilon_m
        ):
            return
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn(
                "Gripper action server %s is not ready" % self.gripper_action_name
            )
            return

        goal = Move.Goal()
        goal.width = width
        goal.speed = self.gripper_speed_mps
        self.gripper_client.send_goal_async(goal)
        self.last_gripper_command_time = now
        self.last_gripper_width_command = width


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OpenVlaOftInferenceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
