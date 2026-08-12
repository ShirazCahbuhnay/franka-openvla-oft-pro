"""Trim rosbag2 episodes to the common time window of required VLA topics.

Run this on a ROS 2 machine with ``rosbag2_py`` available. The tool copies
serialized messages without deserializing them, so it does not need the Franka
message packages.
"""

from __future__ import annotations

import argparse
import shutil
import tempfile
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence


DEFAULT_CAMERA_TOPIC = "/vla_front_camera/image_raw"
DEFAULT_JOINT_TOPIC = "/joint_states"
DEFAULT_ROBOT_STATE_TOPIC = "/franka_robot_state_broadcaster/robot_state"
DEFAULT_ACTION_TOPIC = "/servo_node/delta_twist_cmds"
DEFAULT_GRIPPER_STATE_TOPIC = DEFAULT_JOINT_TOPIC
DEFAULT_INSTRUCTION_TOPIC = "/vla/language_instruction"


@dataclass(frozen=True)
class TrimWindow:
    start_ns: int
    end_ns: int
    topic_first_ns: dict[str, int]
    topic_last_ns: dict[str, int]


@dataclass(frozen=True)
class TrimResult:
    input_message_count: int
    output_message_count: int
    per_topic_in: dict[str, int]
    per_topic_out: dict[str, int]
    preserved_pre_window: dict[str, int]


def _import_rosbag2_py():
    try:
        import rosbag2_py  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "Missing rosbag2_py. Run this on the ROS 2 machine after sourcing "
            "ROS and the workspace.\n"
            "The original import error was:\n"
            f"  {exc}"
        ) from exc

    return rosbag2_py


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


def _open_reader(rosbag2_py, bag_uri: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def _topic_metadata(rosbag2_py, topic):
    serialization_format = getattr(topic, "serialization_format", "cdr") or "cdr"
    offered_qos_profiles = getattr(topic, "offered_qos_profiles", "")

    try:
        return rosbag2_py.TopicMetadata(
            name=topic.name,
            type=topic.type,
            serialization_format=serialization_format,
            offered_qos_profiles=offered_qos_profiles,
        )
    except TypeError:
        return rosbag2_py.TopicMetadata(
            name=topic.name,
            type=topic.type,
            serialization_format=serialization_format,
        )


def _collect_topic_times(
    rosbag2_py,
    bag_uri: Path,
    required_topics: set[str],
) -> tuple[dict[str, list[int]], list[object]]:
    reader = _open_reader(rosbag2_py, bag_uri)
    topics = list(reader.get_all_topics_and_types())
    available_topics = {topic.name for topic in topics}
    missing = sorted(required_topics - available_topics)
    if missing:
        raise ValueError(f"Input bag is missing required topic(s): {', '.join(missing)}")

    topic_times = {topic: [] for topic in required_topics}
    while reader.has_next():
        topic, _, timestamp_ns = reader.read_next()
        if topic in topic_times:
            topic_times[topic].append(timestamp_ns)

    empty = sorted(topic for topic, times in topic_times.items() if not times)
    if empty:
        raise ValueError(f"Input bag has no messages for topic(s): {', '.join(empty)}")

    return topic_times, topics


def _compute_trim_window(
    topic_times: dict[str, list[int]],
    *,
    start_pad_sec: float,
    end_pad_sec: float,
) -> TrimWindow:
    topic_first_ns = {topic: times[0] for topic, times in topic_times.items()}
    topic_last_ns = {topic: times[-1] for topic, times in topic_times.items()}
    start_ns = max(topic_first_ns.values()) + int(start_pad_sec * 1e9)
    end_ns = min(topic_last_ns.values()) - int(end_pad_sec * 1e9)

    if start_ns >= end_ns:
        raise ValueError(
            "No overlapping time window remains after padding. "
            "Try smaller --start-pad-sec/--end-pad-sec values."
        )

    return TrimWindow(
        start_ns=start_ns,
        end_ns=end_ns,
        topic_first_ns=topic_first_ns,
        topic_last_ns=topic_last_ns,
    )


def _create_writer(rosbag2_py, output_dir: Path, topics: Sequence[object]):
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    for topic in topics:
        writer.create_topic(_topic_metadata(rosbag2_py, topic))
    return writer


def _messages_to_preserve(
    rosbag2_py,
    *,
    bag_uri: Path,
    window: TrimWindow,
    preserve_topics: set[str],
) -> dict[str, object]:
    if not preserve_topics:
        return {}

    reader = _open_reader(rosbag2_py, bag_uri)
    latest_before_start: dict[str, object] = {}
    has_message_in_window = {topic: False for topic in preserve_topics}

    while reader.has_next():
        topic, serialized_msg, timestamp_ns = reader.read_next()
        if topic not in preserve_topics:
            continue
        if timestamp_ns < window.start_ns:
            latest_before_start[topic] = serialized_msg
        elif window.start_ns <= timestamp_ns <= window.end_ns:
            has_message_in_window[topic] = True

    return {
        topic: serialized_msg
        for topic, serialized_msg in latest_before_start.items()
        if not has_message_in_window.get(topic, False)
    }


def _copy_window(
    rosbag2_py,
    *,
    bag_uri: Path,
    output_dir: Path,
    topics: Sequence[object],
    window: TrimWindow,
    preserve_topics: set[str],
) -> TrimResult:
    preserved_messages = _messages_to_preserve(
        rosbag2_py,
        bag_uri=bag_uri,
        window=window,
        preserve_topics=preserve_topics,
    )
    reader = _open_reader(rosbag2_py, bag_uri)
    writer = _create_writer(rosbag2_py, output_dir, topics)
    per_topic_in: dict[str, int] = {}
    per_topic_out: dict[str, int] = {}
    input_message_count = 0
    output_message_count = 0
    preserved_pre_window: dict[str, int] = {}

    for topic, serialized_msg in preserved_messages.items():
        writer.write(topic, serialized_msg, window.start_ns)
        output_message_count += 1
        per_topic_out[topic] = per_topic_out.get(topic, 0) + 1
        preserved_pre_window[topic] = preserved_pre_window.get(topic, 0) + 1

    while reader.has_next():
        topic, serialized_msg, timestamp_ns = reader.read_next()
        input_message_count += 1
        per_topic_in[topic] = per_topic_in.get(topic, 0) + 1
        if window.start_ns <= timestamp_ns <= window.end_ns:
            writer.write(topic, serialized_msg, timestamp_ns)
            output_message_count += 1
            per_topic_out[topic] = per_topic_out.get(topic, 0) + 1

    return TrimResult(
        input_message_count=input_message_count,
        output_message_count=output_message_count,
        per_topic_in=per_topic_in,
        per_topic_out=per_topic_out,
        preserved_pre_window=preserved_pre_window,
    )


def _format_sec(ns: int) -> str:
    return f"{ns / 1e9:.6f}s"


def _print_summary(
    *,
    input_path: Path,
    output_dir: Path,
    required_topics: Sequence[str],
    window: TrimWindow,
    result: TrimResult,
) -> None:
    print(f"Input bag: {input_path.expanduser()}")
    print(f"Output bag: {output_dir}")
    print(f"Trim window: {_format_sec(window.start_ns)} -> {_format_sec(window.end_ns)}")
    print(f"Window duration: {(window.end_ns - window.start_ns) / 1e9:.3f}s")
    print(
        "Messages copied: "
        f"{result.output_message_count}/{result.input_message_count}"
    )
    print("")
    print("Required topic overlap:")
    for topic in required_topics:
        first_ns = window.topic_first_ns[topic]
        last_ns = window.topic_last_ns[topic]
        print(
            f"  {topic}: "
            f"{_format_sec(first_ns)} -> {_format_sec(last_ns)}; "
            f"kept {result.per_topic_out.get(topic, 0)}/"
            f"{result.per_topic_in.get(topic, 0)}"
        )
    if result.preserved_pre_window:
        print("")
        print("Preserved latest pre-window message(s):")
        for topic, count in sorted(result.preserved_pre_window.items()):
            print(f"  {topic}: {count} message(s) written at trim start")


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Trim a rosbag2 episode to the common timestamp overlap of VLA "
            "camera, robot-state, joint-state, and action topics."
        )
    )
    parser.add_argument(
        "bag",
        type=Path,
        help="Input rosbag2 directory, .db3 file, or zipped rosbag2 directory.",
    )
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="Output rosbag2 directory to create.",
    )
    parser.add_argument("--camera-topic", default=DEFAULT_CAMERA_TOPIC)
    parser.add_argument("--joint-topic", default=DEFAULT_JOINT_TOPIC)
    parser.add_argument("--robot-state-topic", default=DEFAULT_ROBOT_STATE_TOPIC)
    parser.add_argument("--action-topic", default=DEFAULT_ACTION_TOPIC)
    parser.add_argument(
        "--instruction-topic",
        default=DEFAULT_INSTRUCTION_TOPIC,
        help=(
            "Instruction topic to preserve if its first message falls before "
            "the synchronized trim window."
        ),
    )
    parser.add_argument(
        "--gripper-state-topic",
        default=DEFAULT_GRIPPER_STATE_TOPIC,
        help=(
            "Topic used by rosbag2_to_rlds to infer OpenVLA-OFT gripper "
            "actions. Defaults to --joint-topic for the current recorder."
        ),
    )
    parser.add_argument(
        "--extra-required-topic",
        action="append",
        default=[],
        help=(
            "Additional topic that must overlap the trim window. May be passed "
            "multiple times."
        ),
    )
    parser.add_argument(
        "--preserve-latest-before-start-topic",
        action="append",
        default=[],
        help=(
            "Copy the latest message before the trim start for this topic if "
            "the topic has no message inside the trim window. May be passed "
            "multiple times. The instruction topic is preserved this way by "
            "default unless --no-preserve-instruction is set."
        ),
    )
    parser.add_argument(
        "--no-preserve-instruction",
        action="store_true",
        help="Do not preserve a pre-window instruction message at the trim start.",
    )
    parser.add_argument(
        "--start-pad-sec",
        type=float,
        default=0.0,
        help="Drop this many extra seconds after the latest required topic starts.",
    )
    parser.add_argument(
        "--end-pad-sec",
        type=float,
        default=0.0,
        help="Drop this many extra seconds before the earliest required topic ends.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite the output bag directory if it already exists.",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
    args = _parse_args(argv)
    if args.start_pad_sec < 0.0 or args.end_pad_sec < 0.0:
        raise SystemExit("--start-pad-sec and --end-pad-sec must be non-negative")

    output_dir = args.output.expanduser().resolve()
    if output_dir.exists():
        if not args.overwrite:
            raise SystemExit(f"Output already exists: {output_dir}")
        shutil.rmtree(output_dir)

    required_topics = list(dict.fromkeys([
        args.camera_topic,
        args.robot_state_topic,
        args.joint_topic,
        args.action_topic,
        args.gripper_state_topic,
        *args.extra_required_topic,
    ]))
    required_topic_set = set(required_topics)
    preserve_topics = list(args.preserve_latest_before_start_topic)
    if not args.no_preserve_instruction:
        preserve_topics.append(args.instruction_topic)
    preserve_topic_set = set(topic for topic in preserve_topics if topic)
    rosbag2_py = _import_rosbag2_py()

    with tempfile.TemporaryDirectory(prefix="rosbag2_trim_sync_") as temp_name:
        bag_uri = _bag_uri_from_input(args.bag, Path(temp_name))
        topic_times, topics = _collect_topic_times(
            rosbag2_py,
            bag_uri,
            required_topic_set,
        )
        window = _compute_trim_window(
            topic_times,
            start_pad_sec=args.start_pad_sec,
            end_pad_sec=args.end_pad_sec,
        )
        result = _copy_window(
            rosbag2_py,
            bag_uri=bag_uri,
            output_dir=output_dir,
            topics=topics,
            window=window,
            preserve_topics=preserve_topic_set,
        )

    _print_summary(
        input_path=args.bag,
        output_dir=output_dir,
        required_topics=required_topics,
        window=window,
        result=result,
    )


if __name__ == "__main__":
    main()
