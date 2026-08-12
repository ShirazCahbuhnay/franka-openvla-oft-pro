"""Convert Franka VLA ROS 2 bag episodes into an RLDS/TFDS dataset.

Run this on a ROS 2 machine where the Franka message packages are sourced.
The converter accepts rosbag2 directories, `.db3` files, or zipped rosbag2
directories.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import re
import tempfile
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence

import numpy as np


DEFAULT_CAMERA_TOPIC = "/vla_front_camera/image_raw"
DEFAULT_JOINT_TOPIC = "/joint_states"
DEFAULT_ROBOT_STATE_TOPIC = "/franka_robot_state_broadcaster/robot_state"
DEFAULT_ACTION_TOPIC = "/servo_node/delta_twist_cmds"
DEFAULT_INSTRUCTION_TOPIC = "/vla/language_instruction"
DEFAULT_GRIPPER_STATE_TOPIC = DEFAULT_JOINT_TOPIC
DEFAULT_MAX_GRIPPER_WIDTH_M = 0.08
DEFAULT_GRIPPER_BINARY_MOTION_THRESHOLD_M = 0.001


@dataclass(frozen=True)
class ConverterConfig:
    camera_topic: str
    joint_topic: str
    robot_state_topic: str
    action_topic: str
    instruction_topic: str
    gripper_state_topic: str
    image_size: int
    resize_mode: str
    max_sync_dt_sec: float
    twist_action_mode: str
    gripper_action_mode: str
    max_gripper_width_m: float
    gripper_binary_motion_threshold_m: float
    default_instruction: str | None
    instruction_overrides: dict[str, str]
    embodiment: str
    success: bool


@dataclass(frozen=True)
class TimedMessage:
    timestamp_ns: int
    msg: object


def _import_runtime_dependencies():
    try:
        import cv2  # type: ignore
        import rosbag2_py  # type: ignore
        import tensorflow_datasets as tfds  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "Missing converter dependency. Run this on the ROS 2 machine after "
            "sourcing ROS and the workspace, then install Python packages with:\n"
            "  python3 -m pip install -r "
            "src/franka_ros2/vla_data_collection/requirements-rlds.txt\n"
            "The original import error was:\n"
            f"  {exc}"
        ) from exc

    return cv2, rosbag2_py, tfds, deserialize_message, get_message


def _safe_extract_zip(zip_path: Path, dst: Path) -> None:
    root = dst.resolve()
    with zipfile.ZipFile(zip_path) as zf:
        for member in zf.infolist():
            target = (dst / member.filename).resolve()
            if root not in target.parents and target != root:
                raise ValueError(f"Refusing to extract unsafe zip member: {member.filename}")
        zf.extractall(dst)


def _bag_uri_from_input(path: Path, temp_dir: Path) -> Path:
    path = path.expanduser().resolve()
    if path.suffix == ".zip":
        extract_dir = temp_dir / path.stem
        extract_dir.mkdir(parents=True, exist_ok=True)
        _safe_extract_zip(path, extract_dir)
        metadata_files = sorted(extract_dir.rglob("metadata.yaml"))
        if not metadata_files:
            raise FileNotFoundError(f"No metadata.yaml found inside {path}")
        return metadata_files[0].parent

    if path.suffix == ".db3":
        return path.parent

    if path.is_dir():
        return path

    raise ValueError(f"Expected a rosbag2 directory, .db3 file, or .zip file: {path}")


def _episode_id_from_path(path: Path) -> str:
    return path.stem if path.suffix else path.name


def _canonical_input_key(path: Path) -> str:
    return str(path.expanduser().resolve())


def _instruction_from_episode_id(episode_id: str) -> str:
    task = re.sub(r"^episode[_-]?\d+[_-]?", "", episode_id)
    task = task.replace("_", " ").replace("-", " ").strip()
    return task or episode_id


def _clean_instruction(text: object | None) -> str | None:
    if text is None:
        return None
    cleaned = str(text).strip()
    return cleaned or None


def _resolve_instruction(
    *,
    default_instruction: str | None,
    instructions: Sequence[TimedMessage],
    episode_id: str,
) -> str:
    for candidate in (default_instruction,):
        cleaned = _clean_instruction(candidate)
        if cleaned:
            return cleaned

    for instruction_item in instructions:
        cleaned = _clean_instruction(getattr(instruction_item.msg, "data", None))
        if cleaned:
            return cleaned

    return _instruction_from_episode_id(episode_id)


def _instruction_override_for_path(path: Path, config: ConverterConfig) -> str | None:
    return config.instruction_overrides.get(_canonical_input_key(path))


def _load_instruction_map(path: Path) -> dict[str, str]:
    map_path = path.expanduser().resolve()
    with map_path.open("r", encoding="utf-8") as f:
        raw = json.load(f)

    if not isinstance(raw, dict):
        raise ValueError("--instruction-map must be a JSON object of bag path to instruction")

    overrides: dict[str, str] = {}
    for raw_bag_path, raw_instruction in raw.items():
        if not isinstance(raw_bag_path, str):
            raise ValueError("--instruction-map keys must be bag paths")

        instruction = _clean_instruction(raw_instruction)
        if not instruction:
            raise ValueError(
                "Empty instruction for bag path in --instruction-map: "
                f"{raw_bag_path}"
            )

        bag_path = Path(raw_bag_path).expanduser()
        if not bag_path.is_absolute():
            bag_path = map_path.parent / bag_path
        overrides[_canonical_input_key(bag_path)] = instruction

    return overrides


def _read_bag_streams(path: Path, config: ConverterConfig) -> dict[str, list[TimedMessage]]:
    _, rosbag2_py, _, deserialize_message, get_message = _import_runtime_dependencies()
    required_topics = {
        config.camera_topic,
        config.joint_topic,
        config.robot_state_topic,
        config.action_topic,
        config.gripper_state_topic,
    }
    wanted_topics = set(required_topics)
    wanted_topics.add(config.instruction_topic)

    with tempfile.TemporaryDirectory(prefix="franka_rlds_bag_") as temp_name:
        bag_uri = _bag_uri_from_input(path, Path(temp_name))
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag_uri), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )

        topic_types = {
            topic.name: topic.type for topic in reader.get_all_topics_and_types()
        }
        missing = sorted(required_topics - topic_types.keys())
        if config.default_instruction is None and config.instruction_topic not in topic_types:
            missing.append(config.instruction_topic)
        if missing:
            raise ValueError(
                f"{path} is missing required topic(s): {', '.join(missing)}"
            )

        message_types = {
            topic: get_message(topic_types[topic])
            for topic in wanted_topics
            if topic in topic_types
        }
        streams: dict[str, list[TimedMessage]] = {topic: [] for topic in wanted_topics}
        while reader.has_next():
            topic, serialized_msg, timestamp_ns = reader.read_next()
            if topic not in message_types:
                continue
            msg = deserialize_message(serialized_msg, message_types[topic])
            streams[topic].append(TimedMessage(timestamp_ns, msg))

    return streams


def _times(stream: Sequence[TimedMessage]) -> list[int]:
    return [item.timestamp_ns for item in stream]


def _nearest(
    stream: Sequence[TimedMessage],
    timestamp_ns: int,
    max_dt_ns: int | None,
) -> TimedMessage | None:
    if not stream:
        return None

    timestamps = _times(stream)
    insert_at = bisect.bisect_left(timestamps, timestamp_ns)
    candidates = []
    if insert_at > 0:
        candidates.append(stream[insert_at - 1])
    if insert_at < len(stream):
        candidates.append(stream[insert_at])
    best = min(candidates, key=lambda item: abs(item.timestamp_ns - timestamp_ns))

    if max_dt_ns is not None and abs(best.timestamp_ns - timestamp_ns) > max_dt_ns:
        return None
    return best


def _twist_vector(msg: object) -> np.ndarray:
    twist = msg.twist
    return np.array(
        [
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        ],
        dtype=np.float64,
    )


def _mean_twist(
    twist_stream: Sequence[TimedMessage],
    start_ns: int,
    end_ns: int,
    max_dt_ns: int,
) -> np.ndarray | None:
    if not twist_stream:
        return None

    timestamps = _times(twist_stream)
    left = bisect.bisect_left(timestamps, start_ns)
    right = bisect.bisect_left(timestamps, end_ns)
    if right > left:
        return np.mean([_twist_vector(item.msg) for item in twist_stream[left:right]], axis=0)

    nearest = _nearest(twist_stream, start_ns, max_dt_ns)
    if nearest is None:
        return None
    return _twist_vector(nearest.msg)


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


def _eef_xyz_rpy(robot_state_msg: object) -> np.ndarray:
    transform = np.asarray(robot_state_msg.o_t_ee, dtype=np.float64).reshape(4, 4, order="F")
    xyz = transform[:3, 3].astype(np.float32)
    rpy = _rotation_matrix_to_rpy(transform[:3, :3])
    return np.concatenate([xyz, rpy]).astype(np.float32)


def _gripper_width(joint_state_msg: object | None) -> float:
    if joint_state_msg is None:
        return 0.0

    names = list(getattr(joint_state_msg, "name", []))
    positions = list(getattr(joint_state_msg, "position", []))
    width = 0.0
    found = False
    for name, position in zip(names, positions):
        lowered = name.lower()
        if "finger" in lowered or "gripper" in lowered:
            width += float(position)
            found = True
    return width if found else 0.0


def _normalized_gripper_width(width: float, max_width: float) -> float:
    if max_width <= 0.0:
        raise ValueError("--max-gripper-width-m must be greater than 0")
    return float(np.clip(width / max_width, 0.0, 1.0))


def _gripper_action_convention(mode: str) -> str:
    if mode == "absolute_binary":
        return "OpenVLA-OFT absolute gripper: 0.0=closed, 1.0=open"
    if mode == "absolute_width":
        return "Normalized physical gripper width: 0.0=closed, 1.0=max width"
    if mode == "delta":
        return "Physical gripper width delta in meters"
    return mode


def _gripper_widths_from_steps(steps: Sequence[dict]) -> np.ndarray:
    return np.asarray(
        [float(step["observation"]["gripper_width"]) for step in steps],
        dtype=np.float64,
    )


def _infer_absolute_binary_gripper_actions(
    widths: np.ndarray,
    *,
    motion_threshold_m: float,
) -> np.ndarray:
    """Infer OpenVLA-style absolute open/closed commands from measured width.

    The Franka gripper may stop at the object width while still executing a
    "close" command. Therefore this uses width direction changes, not only
    the physical width value, and carries the last moving command through
    stationary contact periods.
    """

    if widths.size == 0:
        return np.asarray([], dtype=np.float32)
    if motion_threshold_m < 0.0:
        raise ValueError("--gripper-binary-motion-threshold-m must be non-negative")

    observed_min = float(np.min(widths))
    observed_max = float(np.max(widths))
    if observed_max - observed_min <= motion_threshold_m:
        command = 1.0 if observed_max > motion_threshold_m else 0.0
        return np.full(widths.shape[0], command, dtype=np.float32)

    observed_mid = observed_min + 0.5 * (observed_max - observed_min)
    command = 1.0 if float(widths[0]) >= observed_mid else 0.0
    actions = np.empty(widths.shape[0], dtype=np.float32)

    for index in range(widths.shape[0]):
        if index + 1 < widths.shape[0]:
            delta = float(widths[index + 1] - widths[index])
            if delta > motion_threshold_m:
                command = 1.0
            elif delta < -motion_threshold_m:
                command = 0.0
        actions[index] = command

    return actions


def _apply_gripper_actions(steps: list[dict], config: ConverterConfig) -> None:
    widths = _gripper_widths_from_steps(steps)
    if widths.size == 0:
        return

    next_widths = np.concatenate([widths[1:], widths[-1:]])
    if config.gripper_action_mode == "absolute_binary":
        gripper_actions = _infer_absolute_binary_gripper_actions(
            widths,
            motion_threshold_m=config.gripper_binary_motion_threshold_m,
        )
    elif config.gripper_action_mode == "absolute_width":
        gripper_actions = np.asarray(
            [
                _normalized_gripper_width(width, config.max_gripper_width_m)
                for width in next_widths
            ],
            dtype=np.float32,
        )
    elif config.gripper_action_mode == "delta":
        gripper_actions = (next_widths - widths).astype(np.float32)
    else:
        raise ValueError(f"Unknown gripper action mode: {config.gripper_action_mode}")

    for step, gripper_action in zip(steps, gripper_actions):
        step["action"][-1] = np.float32(gripper_action)


def _finalize_step_flags(steps: list[dict], success: bool) -> None:
    last_index = len(steps) - 1
    for index, step in enumerate(steps):
        is_last = index == last_index
        step["reward"] = np.float32(1.0 if is_last and success else 0.0)
        step["discount"] = np.float32(0.0 if is_last else 1.0)
        step["is_first"] = index == 0
        step["is_last"] = is_last
        step["is_terminal"] = is_last


def _resize_image(cv2, image: np.ndarray, image_size: int, resize_mode: str) -> np.ndarray:
    if resize_mode == "center_crop":
        height, width = image.shape[:2]
        crop = min(height, width)
        y0 = (height - crop) // 2
        x0 = (width - crop) // 2
        image = image[y0 : y0 + crop, x0 : x0 + crop]

    resized = cv2.resize(image, (image_size, image_size), interpolation=cv2.INTER_AREA)
    return resized.astype(np.uint8)


def _strided_image_array(data: np.ndarray, height: int, width: int, step: int, channels: int) -> np.ndarray:
    expected_row_bytes = width * channels
    if height <= 0 or width <= 0:
        raise ValueError(f"Invalid raw image shape: height={height}, width={width}")
    if step < expected_row_bytes:
        raise ValueError(
            f"Raw image step {step} is too small for width={width}, channels={channels}"
        )
    if data.size < height * step:
        raise ValueError(
            f"Raw image data has {data.size} byte(s), expected at least {height * step}"
        )

    rows = data[: height * step].reshape(height, step)
    compact = rows[:, :expected_row_bytes]
    return compact.reshape(height, width, channels)


def _decode_compressed_image(cv2, compressed_image_msg: object) -> np.ndarray:
    encoded = np.frombuffer(bytes(compressed_image_msg.data), dtype=np.uint8)
    bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("OpenCV could not decode a CompressedImage message")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def _decode_raw_image(cv2, image_msg: object) -> np.ndarray:
    height = int(getattr(image_msg, "height"))
    width = int(getattr(image_msg, "width"))
    step = int(getattr(image_msg, "step"))
    encoding = str(getattr(image_msg, "encoding", "")).lower().replace("-", "_")
    data = np.frombuffer(bytes(image_msg.data), dtype=np.uint8)

    if encoding in {"rgb8", "8uc3"}:
        return _strided_image_array(data, height, width, step, 3).copy()
    if encoding == "bgr8":
        bgr = _strided_image_array(data, height, width, step, 3)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    if encoding in {"rgba8", "rgb8a"}:
        rgba = _strided_image_array(data, height, width, step, 4)
        return cv2.cvtColor(rgba, cv2.COLOR_RGBA2RGB)
    if encoding == "bgra8":
        bgra = _strided_image_array(data, height, width, step, 4)
        return cv2.cvtColor(bgra, cv2.COLOR_BGRA2RGB)
    if encoding in {"mono8", "8uc1"}:
        mono = _strided_image_array(data, height, width, step, 1)
        return cv2.cvtColor(mono, cv2.COLOR_GRAY2RGB)
    if encoding in {"yuyv", "yuy2", "yuv422_yuy2", "yuv422"}:
        yuyv = _strided_image_array(data, height, width, step, 2)
        return cv2.cvtColor(yuyv, cv2.COLOR_YUV2RGB_YUY2)
    if encoding in {"uyvy", "yuv422_uyvy"}:
        uyvy = _strided_image_array(data, height, width, step, 2)
        return cv2.cvtColor(uyvy, cv2.COLOR_YUV2RGB_UYVY)

    raise ValueError(
        f"Unsupported raw Image encoding {encoding!r}. "
        "Supported encodings include rgb8, bgr8, mono8, rgba8, bgra8, yuyv/yuy2, and uyvy."
    )


def _decode_image(cv2, image_msg: object, image_size: int, resize_mode: str) -> np.ndarray:
    if hasattr(image_msg, "format"):
        rgb = _decode_compressed_image(cv2, image_msg)
    elif hasattr(image_msg, "encoding"):
        rgb = _decode_raw_image(cv2, image_msg)
    else:
        raise ValueError(
            "Camera message is neither sensor_msgs/CompressedImage nor sensor_msgs/Image"
        )
    return _resize_image(cv2, rgb, image_size, resize_mode)


def _make_step(
    *,
    cv2,
    image_item: TimedMessage,
    next_image_time_ns: int,
    image_index: int,
    last_image_index: int,
    robot_state_item: TimedMessage,
    gripper_state_item: TimedMessage,
    mean_twist: np.ndarray,
    instruction: str,
    config: ConverterConfig,
) -> dict:
    dt_sec = max((next_image_time_ns - image_item.timestamp_ns) / 1e9, 0.0)
    eef_xyz_rpy = _eef_xyz_rpy(robot_state_item.msg)
    gripper_width = _gripper_width(gripper_state_item.msg)

    if config.twist_action_mode == "integrated_delta":
        arm_action = mean_twist * dt_sec
    else:
        arm_action = mean_twist
    action = np.concatenate([arm_action, np.array([0.0])]).astype(np.float32)

    joint_position = np.asarray(robot_state_item.msg.q, dtype=np.float32)
    joint_velocity = np.asarray(robot_state_item.msg.dq, dtype=np.float32)
    state = np.concatenate(
        [eef_xyz_rpy, np.array([0.0, gripper_width], dtype=np.float32)]
    ).astype(np.float32)
    is_last = image_index == last_image_index

    return {
        "observation": {
            "image": _decode_image(
                cv2,
                image_item.msg,
                config.image_size,
                config.resize_mode,
            ),
            "state": state,
            "eef_pose": eef_xyz_rpy,
            "joint_position": joint_position,
            "joint_velocity": joint_velocity,
            "gripper_width": np.float32(gripper_width),
        },
        "action": action,
        "reward": np.float32(1.0 if is_last and config.success else 0.0),
        "discount": np.float32(0.0 if is_last else 1.0),
        "is_first": image_index == 0,
        "is_last": is_last,
        "is_terminal": is_last,
        "language_instruction": instruction,
        "timestamp_ns": np.int64(image_item.timestamp_ns),
    }


def _episode_from_bag(path: Path, config: ConverterConfig) -> dict:
    cv2, _, _, _, _ = _import_runtime_dependencies()
    streams = _read_bag_streams(path, config)
    images = streams[config.camera_topic]
    robot_states = streams[config.robot_state_topic]
    gripper_states = streams[config.gripper_state_topic]
    twists = streams[config.action_topic]
    instructions = streams[config.instruction_topic]

    if len(images) < 2:
        raise ValueError(f"{path} does not contain enough camera frames")

    max_dt_ns = int(config.max_sync_dt_sec * 1e9)
    episode_id = _episode_id_from_path(path)
    explicit_instruction = (
        _instruction_override_for_path(path, config) or config.default_instruction
    )
    instruction = _resolve_instruction(
        default_instruction=explicit_instruction,
        instructions=instructions,
        episode_id=episode_id,
    )

    steps = []
    last_image_index = len(images) - 2
    for image_index, image_item in enumerate(images[:-1]):
        next_image_time_ns = images[image_index + 1].timestamp_ns
        robot_state_item = _nearest(robot_states, image_item.timestamp_ns, max_dt_ns)
        gripper_state_item = _nearest(gripper_states, image_item.timestamp_ns, max_dt_ns)
        mean_twist = _mean_twist(twists, image_item.timestamp_ns, next_image_time_ns, max_dt_ns)
        if robot_state_item is None or gripper_state_item is None or mean_twist is None:
            continue

        steps.append(
            _make_step(
                cv2=cv2,
                image_item=image_item,
                next_image_time_ns=next_image_time_ns,
                image_index=image_index,
                last_image_index=last_image_index,
                robot_state_item=robot_state_item,
                gripper_state_item=gripper_state_item,
                mean_twist=mean_twist,
                instruction=instruction,
                config=config,
            )
        )

    if not steps:
        raise ValueError(f"No synchronized RLDS steps could be produced from {path}")

    _apply_gripper_actions(steps, config)
    _finalize_step_flags(steps, config.success)

    return {
        "steps": steps,
        "episode_metadata": {
            "file_path": str(path.expanduser()),
            "episode_id": episode_id,
            "embodiment": config.embodiment,
            "camera_topic": config.camera_topic,
            "action_topic": config.action_topic,
            "gripper_state_topic": config.gripper_state_topic,
            "gripper_action_mode": config.gripper_action_mode,
            "gripper_action_convention": _gripper_action_convention(
                config.gripper_action_mode
            ),
            "max_gripper_width_m": np.float32(config.max_gripper_width_m),
            "gripper_binary_motion_threshold_m": np.float32(
                config.gripper_binary_motion_threshold_m
            ),
            "success": config.success,
        },
    }


_DATASET_NAME_RE = re.compile(r"^[a-z][a-z0-9_]*$")


def _dataset_name_to_class_name(dataset_name: str) -> str:
    cleaned = dataset_name.strip()
    if not _DATASET_NAME_RE.fullmatch(cleaned):
        raise ValueError(
            "--dataset-name must be snake_case and start with a lowercase letter, "
            "for example panda_vla_dataset"
        )
    return "".join(part.capitalize() for part in cleaned.split("_"))


def _safe_example_key(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", text).strip("_") or "episode"


def _unique_example_key(base_key: str, path: Path, used_keys: set[str]) -> str:
    safe_base = _safe_example_key(base_key)
    if safe_base not in used_keys:
        used_keys.add(safe_base)
        return safe_base

    parent = _safe_example_key(path.parent.name)
    candidate = f"{parent}_{safe_base}"
    if candidate not in used_keys:
        used_keys.add(candidate)
        return candidate

    index = 2
    while f"{candidate}_{index}" in used_keys:
        index += 1

    unique_key = f"{candidate}_{index}"
    used_keys.add(unique_key)
    return unique_key


def _make_builder_class(
    tfds,
    bag_paths: Sequence[Path],
    config: ConverterConfig,
    dataset_name: str | None = None,
    episode_transform: Callable[[dict], dict] | None = None,
):
    class FrankaVla(tfds.core.GeneratorBasedBuilder):
        VERSION = tfds.core.Version("1.0.0")
        RELEASE_NOTES = {"1.0.0": "Initial Franka VLA rosbag2 conversion."}

        def _info(self):
            return tfds.core.DatasetInfo(
                builder=self,
                description=(
                    "Franka robot manipulation demonstrations converted from "
                    "ROS 2 bags for VLA/OpenVLA fine-tuning."
                ),
                features=tfds.features.FeaturesDict(
                    {
                        "steps": tfds.features.Dataset(
                            {
                                "observation": tfds.features.FeaturesDict(
                                    {
                                        "image": tfds.features.Image(
                                            shape=(
                                                config.image_size,
                                                config.image_size,
                                                3,
                                            ),
                                            dtype=np.uint8,
                                        ),
                                        "state": tfds.features.Tensor(
                                            shape=(8,),
                                            dtype=np.float32,
                                        ),
                                        "eef_pose": tfds.features.Tensor(
                                            shape=(6,),
                                            dtype=np.float32,
                                        ),
                                        "joint_position": tfds.features.Tensor(
                                            shape=(7,),
                                            dtype=np.float32,
                                        ),
                                        "joint_velocity": tfds.features.Tensor(
                                            shape=(7,),
                                            dtype=np.float32,
                                        ),
                                        "gripper_width": tfds.features.Scalar(
                                            dtype=np.float32,
                                        ),
                                    }
                                ),
                                "action": tfds.features.Tensor(
                                    shape=(7,),
                                    dtype=np.float32,
                                ),
                                "reward": tfds.features.Scalar(dtype=np.float32),
                                "discount": tfds.features.Scalar(dtype=np.float32),
                                "is_first": tfds.features.Scalar(dtype=np.bool_),
                                "is_last": tfds.features.Scalar(dtype=np.bool_),
                                "is_terminal": tfds.features.Scalar(dtype=np.bool_),
                                "language_instruction": tfds.features.Text(),
                                "timestamp_ns": tfds.features.Scalar(dtype=np.int64),
                            }
                        ),
                        "episode_metadata": tfds.features.FeaturesDict(
                            {
                                "file_path": tfds.features.Text(),
                                "episode_id": tfds.features.Text(),
                                "embodiment": tfds.features.Text(),
                                "camera_topic": tfds.features.Text(),
                                "action_topic": tfds.features.Text(),
                                "gripper_state_topic": tfds.features.Text(),
                                "gripper_action_mode": tfds.features.Text(),
                                "gripper_action_convention": tfds.features.Text(),
                                "max_gripper_width_m": tfds.features.Scalar(
                                    dtype=np.float32,
                                ),
                                "gripper_binary_motion_threshold_m": tfds.features.Scalar(
                                    dtype=np.float32,
                                ),
                                "success": tfds.features.Scalar(dtype=np.bool_),
                            }
                        ),
                    }
                ),
            )

        def _split_generators(self, dl_manager):
            return {"train": self._generate_examples()}

        def _generate_examples(self):
            used_keys: set[str] = set()
            for bag_path in bag_paths:
                episode = _episode_from_bag(bag_path, config)
                if episode_transform is not None:
                    episode = episode_transform(episode)
                episode_id = episode["episode_metadata"]["episode_id"]
                yield _unique_example_key(episode_id, bag_path, used_keys), episode

    if dataset_name:
        class_name = _dataset_name_to_class_name(dataset_name)
        FrankaVla.__name__ = class_name
        FrankaVla.__qualname__ = class_name

    return FrankaVla


def convert_bags(
    bags: Sequence[Path],
    *,
    output_dir: Path,
    config: ConverterConfig,
    overwrite: bool = False,
    dataset_name: str | None = None,
    episode_transform: Callable[[dict], dict] | None = None,
) -> Path:
    _, _, tfds, _, _ = _import_runtime_dependencies()
    builder_class = _make_builder_class(
        tfds,
        bags,
        config,
        dataset_name=dataset_name,
        episode_transform=episode_transform,
    )
    builder = builder_class(data_dir=str(output_dir.expanduser()))
    builder.download_and_prepare(
        download_config=tfds.download.DownloadConfig(
            download_mode=(
                tfds.GenerateMode.REUSE_DATASET_IF_EXISTS
                if not overwrite
                else tfds.GenerateMode.FORCE_REDOWNLOAD
            )
        )
    )
    return Path(builder.data_dir)


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert Franka VLA rosbag2 episodes to an RLDS/TFDS dataset."
    )
    parser.add_argument(
        "bags",
        nargs="+",
        type=Path,
        help="rosbag2 directories, .db3 files, or zipped rosbag2 directories.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("~/tensorflow_datasets"),
        help="TFDS data root. The dataset will be written below this directory.",
    )
    parser.add_argument(
        "--dataset-name",
        default=None,
        help=(
            "TFDS dataset name to write under --output-dir. Defaults to franka_vla. "
            "Use snake_case, for example panda_vla_dataset."
        ),
    )
    parser.add_argument(
        "--camera-topic",
        default=DEFAULT_CAMERA_TOPIC,
        help=(
            "Camera topic to convert. Supports sensor_msgs/CompressedImage "
            "and raw sensor_msgs/Image encodings such as rgb8, bgr8, yuyv/yuy2, and uyvy."
        ),
    )
    parser.add_argument("--joint-topic", default=DEFAULT_JOINT_TOPIC)
    parser.add_argument("--robot-state-topic", default=DEFAULT_ROBOT_STATE_TOPIC)
    parser.add_argument("--action-topic", default=DEFAULT_ACTION_TOPIC)
    parser.add_argument("--instruction-topic", default=DEFAULT_INSTRUCTION_TOPIC)
    parser.add_argument(
        "--gripper-state-topic",
        default=DEFAULT_GRIPPER_STATE_TOPIC,
        help=(
            "JointState topic used to read gripper finger width. Defaults to "
            "--joint-topic for the current recorder."
        ),
    )
    parser.add_argument("--image-size", type=int, default=256)
    parser.add_argument(
        "--resize-mode",
        choices=("center_crop", "stretch"),
        default="center_crop",
        help="How to make camera frames square before storing them.",
    )
    parser.add_argument(
        "--max-sync-dt-sec",
        type=float,
        default=0.10,
        help="Drop a frame if the nearest robot/action sample is farther away.",
    )
    parser.add_argument(
        "--twist-action-mode",
        choices=("integrated_delta", "raw_twist"),
        default="integrated_delta",
        help=(
            "integrated_delta stores twist * camera_dt as an approximate "
            "EEF delta action; raw_twist stores the recorded TwistStamped values."
        ),
    )
    parser.add_argument(
        "--gripper-action-mode",
        choices=("absolute_binary", "absolute_width", "delta"),
        default="absolute_binary",
        help=(
            "absolute_binary stores OpenVLA-OFT-style absolute actions "
            "(0=closed, 1=open) inferred from gripper width motion; "
            "absolute_width stores physical width normalized to [0, 1]; "
            "delta stores next_width - current_width for legacy relative "
            "gripper actions."
        ),
    )
    parser.add_argument(
        "--max-gripper-width-m",
        type=float,
        default=DEFAULT_MAX_GRIPPER_WIDTH_M,
        help=(
            "Fully open gripper width in meters used by "
            "--gripper-action-mode absolute_width."
        ),
    )
    parser.add_argument(
        "--gripper-binary-motion-threshold-m",
        type=float,
        default=DEFAULT_GRIPPER_BINARY_MOTION_THRESHOLD_M,
        help=(
            "Minimum gripper width change in meters used to infer open/close "
            "commands for --gripper-action-mode absolute_binary."
        ),
    )
    parser.add_argument(
        "--instruction",
        default=None,
        help="Override the instruction topic for all converted episodes.",
    )
    parser.add_argument(
        "--instruction-map",
        type=Path,
        default=None,
        help=(
            "JSON object mapping bag paths to per-bag instruction overrides. "
            "Per-bag entries take precedence over --instruction."
        ),
    )
    parser.add_argument("--embodiment", default="franka_panda")
    parser.add_argument(
        "--success",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Whether to mark converted demonstrations as successful.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite an existing TFDS dataset directory.",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
    args = _parse_args(argv)
    instruction_overrides = (
        _load_instruction_map(args.instruction_map) if args.instruction_map else {}
    )
    config = ConverterConfig(
        camera_topic=args.camera_topic,
        joint_topic=args.joint_topic,
        robot_state_topic=args.robot_state_topic,
        action_topic=args.action_topic,
        instruction_topic=args.instruction_topic,
        gripper_state_topic=args.gripper_state_topic,
        image_size=args.image_size,
        resize_mode=args.resize_mode,
        max_sync_dt_sec=args.max_sync_dt_sec,
        twist_action_mode=args.twist_action_mode,
        gripper_action_mode=args.gripper_action_mode,
        max_gripper_width_m=args.max_gripper_width_m,
        gripper_binary_motion_threshold_m=args.gripper_binary_motion_threshold_m,
        default_instruction=args.instruction,
        instruction_overrides=instruction_overrides,
        embodiment=args.embodiment,
        success=args.success,
    )

    data_dir = convert_bags(
        args.bags,
        output_dir=args.output_dir,
        config=config,
        overwrite=args.overwrite,
        dataset_name=args.dataset_name,
    )

    print(f"Wrote RLDS/TFDS dataset to: {data_dir}")
    print("Load it with:")
    print(f"  tfds.builder_from_directory({str(data_dir)!r}).as_dataset(split='train')")


if __name__ == "__main__":
    main()
