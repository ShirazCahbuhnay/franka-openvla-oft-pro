"""Build an HTML report comparing rosbag2 timing with converted RLDS data.

This script intentionally avoids ROS 2 Python packages. It reads rosbag2 sqlite
timestamps directly from `.db3` files and reads converted RLDS episodes through
TensorFlow Datasets.
"""

from __future__ import annotations

import argparse
import base64
import bisect
import html
import importlib.metadata
import json
import math
import sqlite3
import statistics
import tempfile
import zipfile
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Sequence


DEFAULT_CAMERA_TOPIC = "/vla_front_camera/image_raw"
DEFAULT_JOINT_TOPIC = "/joint_states"
DEFAULT_ROBOT_STATE_TOPIC = "/franka_robot_state_broadcaster/robot_state"
DEFAULT_ACTION_TOPIC = "/servo_node/delta_twist_cmds"
DEFAULT_INSTRUCTION_TOPIC = "/vla/language_instruction"
DEFAULT_GRIPPER_STATE_TOPIC = DEFAULT_JOINT_TOPIC

ACTION_LABELS = (
    "dx",
    "dy",
    "dz",
    "droll",
    "dpitch",
    "dyaw",
    "gripper (0 closed, 1 open)",
)
PLOT_COLORS = (
    "#2563eb",
    "#dc2626",
    "#059669",
    "#7c3aed",
    "#d97706",
    "#0891b2",
    "#be123c",
    "#4b5563",
)


@dataclass(frozen=True)
class RosbagSummary:
    """Timing-only rosbag2 data read without ROS 2 imports."""

    bag_uri: Path
    db3_files: tuple[Path, ...]
    topic_types: dict[str, str]
    topic_times_ns: dict[str, list[int]]


@dataclass(frozen=True)
class RldsStep:
    """Small RLDS step projection used by the report."""

    index: int
    timestamp_ns: int
    action: list[float]
    reward: float | None
    is_first: bool | None
    is_last: bool | None
    is_terminal: bool | None
    language_instruction: str | None


@dataclass(frozen=True)
class RldsThumbnail:
    """Encoded image preview for one RLDS step."""

    step_index: int
    timestamp_ns: int
    png_base64: str


@dataclass(frozen=True)
class RldsEpisode:
    """Selected RLDS episode and optional image previews."""

    builder_dir: Path
    split: str
    episode_index: int
    episode_id: str
    file_path: str
    metadata: dict[str, Any]
    steps: list[RldsStep]
    thumbnails: list[RldsThumbnail]
    selection_note: str


def _installed_version(package_name: str) -> str:
    try:
        return importlib.metadata.version(package_name)
    except importlib.metadata.PackageNotFoundError:
        return "not installed"


def _version_tuple(version: str) -> tuple[int, ...]:
    parts = []
    for part in version.split("."):
        digits = ""
        for char in part:
            if not char.isdigit():
                break
            digits += char
        if not digits:
            break
        parts.append(int(digits))
    return tuple(parts)


def _requirements_path() -> Path:
    return Path(__file__).resolve().parents[1] / "requirements-rlds.txt"


def _dependency_error_message(original_error: BaseException | None = None) -> str:
    versions = "\n".join(
        f"  {name}=={_installed_version(name)}"
        for name in (
            "tensorflow",
            "tensorflow-datasets",
            "tensorflow-metadata",
            "protobuf",
        )
    )
    details = ""
    if original_error is not None:
        details = f"\nThe original import error was:\n  {original_error}"

    return (
        "Could not load the RLDS/TFDS reader stack. This report does not need "
        "ROS 2, but it does need compatible TensorFlow Datasets packages.\n\n"
        "Reinstall the pinned RLDS dependencies in the environment you use to "
        "run this script:\n"
        "  python3 -m pip install --upgrade --force-reinstall "
        f"-r {_requirements_path()}\n\n"
        "Installed versions seen by this Python:\n"
        f"{versions}"
        f"{details}"
    )


def _preflight_tfds_runtime() -> None:
    protobuf_version = _installed_version("protobuf")
    metadata_version = _installed_version("tensorflow-metadata")
    protobuf_tuple = _version_tuple(protobuf_version)
    metadata_tuple = _version_tuple(metadata_version)

    if not protobuf_tuple or not metadata_tuple:
        return

    if metadata_tuple >= (1, 16) and protobuf_tuple < (5,):
        raise SystemExit(
            _dependency_error_message(
                RuntimeError(
                    "tensorflow-metadata >= 1.16 can require "
                    "google.protobuf.runtime_version, which is not available "
                    "in protobuf < 5. Use the pinned requirements file."
                )
            )
        )


def _import_tfds_runtime():
    _preflight_tfds_runtime()
    try:
        import tensorflow as tf  # type: ignore
        import tensorflow_datasets as tfds  # type: ignore
    except Exception as exc:
        raise SystemExit(_dependency_error_message(exc)) from exc

    if not hasattr(tfds, "builder_from_directory"):
        raise SystemExit(_dependency_error_message())

    return tf, tfds


def _safe_extract_zip(zip_path: Path, dst: Path) -> None:
    root = dst.resolve()
    with zipfile.ZipFile(zip_path) as zf:
        for member in zf.infolist():
            target = (dst / member.filename).resolve()
            if root not in target.parents and target != root:
                raise ValueError(
                    f"Refusing to extract unsafe zip member: {member.filename}"
                )
        zf.extractall(dst)


def _bag_uri_from_input(path: Path, temp_dir: Path) -> Path:
    path = path.expanduser().resolve()
    if path.suffix == ".zip":
        extract_dir = temp_dir / path.stem
        extract_dir.mkdir(parents=True, exist_ok=True)
        _safe_extract_zip(path, extract_dir)
        metadata_files = sorted(extract_dir.rglob("metadata.yaml"))
        if metadata_files:
            return metadata_files[0].parent
        db3_files = sorted(extract_dir.rglob("*.db3"))
        if db3_files:
            return db3_files[0].parent
        raise FileNotFoundError(f"No rosbag2 metadata or .db3 file found in {path}")

    if path.suffix == ".db3":
        return path

    if path.is_dir():
        return path

    raise ValueError(f"Expected a rosbag2 directory, .db3 file, or .zip file: {path}")


def _episode_id_from_path(path: Path) -> str:
    return path.stem if path.suffix else path.name


def _db3_files_for_uri(bag_uri: Path) -> tuple[Path, ...]:
    if bag_uri.suffix == ".db3":
        return (bag_uri,)

    db3_files = tuple(sorted(bag_uri.rglob("*.db3")))
    if not db3_files:
        raise FileNotFoundError(f"No .db3 sqlite files found below {bag_uri}")
    return db3_files


def _read_rosbag2_sqlite(
    bag_uri: Path,
    topics: Sequence[str],
) -> RosbagSummary:
    db3_files = _db3_files_for_uri(bag_uri)
    wanted_topics = set(topics)
    topic_types: dict[str, str] = {}
    topic_times_ns = {topic: [] for topic in wanted_topics}

    for db3_path in db3_files:
        uri = f"file:{db3_path}?mode=ro"
        with sqlite3.connect(uri, uri=True) as conn:
            rows = conn.execute("SELECT id, name, type FROM topics").fetchall()
            topic_rows = {
                int(topic_id): (str(name), str(type_name))
                for topic_id, name, type_name in rows
            }
            for name, type_name in topic_rows.values():
                topic_types.setdefault(name, type_name)

            for topic_id, (name, _type_name) in topic_rows.items():
                if name not in wanted_topics:
                    continue
                cursor = conn.execute(
                    "SELECT timestamp FROM messages "
                    "WHERE topic_id = ? ORDER BY timestamp",
                    (topic_id,),
                )
                topic_times_ns[name].extend(int(row[0]) for row in cursor)

    for times_ns in topic_times_ns.values():
        times_ns.sort()

    return RosbagSummary(
        bag_uri=bag_uri,
        db3_files=db3_files,
        topic_types=topic_types,
        topic_times_ns=topic_times_ns,
    )


def _resolve_tfds_builder_dir(path: Path, dataset_name: str) -> Path:
    path = path.expanduser().resolve()
    if path.is_file():
        path = path.parent

    direct = path / "dataset_info.json"
    if direct.exists():
        return path

    default_version = path / "1.0.0" / "dataset_info.json"
    if default_version.exists():
        return default_version.parent

    common = path / dataset_name / "1.0.0" / "dataset_info.json"
    if common.exists():
        return common.parent

    candidates: list[Path] = []
    if path.exists():
        for dataset_dir in sorted(item for item in path.iterdir() if item.is_dir()):
            if dataset_dir.name != dataset_name and dataset_name:
                continue
            for version_dir in sorted(
                item for item in dataset_dir.iterdir() if item.is_dir()
            ):
                if (version_dir / "dataset_info.json").exists():
                    candidates.append(version_dir)

    if len(candidates) == 1:
        return candidates[0]
    if candidates:
        listed = "\n  ".join(str(candidate) for candidate in candidates)
        raise ValueError(
            "Multiple TFDS builder directories matched. Pass the exact "
            f"`--rlds` version directory:\n  {listed}"
        )

    raise FileNotFoundError(
        f"Could not find dataset_info.json below {path}. Pass the TFDS "
        "builder directory, for example ~/tensorflow_datasets/franka_vla/1.0.0."
    )


def _to_python(value: Any) -> Any:
    if hasattr(value, "numpy"):
        value = value.numpy()
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    if hasattr(value, "item"):
        try:
            return value.item()
        except ValueError:
            pass
    return value


def _to_float(value: Any) -> float | None:
    if value is None:
        return None
    value = _to_python(value)
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _to_bool(value: Any) -> bool | None:
    if value is None:
        return None
    value = _to_python(value)
    if isinstance(value, bool):
        return value
    try:
        return bool(value)
    except TypeError:
        return None


def _array_to_float_list(value: Any) -> list[float]:
    if value is None:
        return []
    value = _to_python(value)
    if hasattr(value, "tolist"):
        value = value.tolist()
    if not isinstance(value, (list, tuple)):
        return []
    return [float(item) for item in value]


def _metadata_to_dict(metadata: dict[str, Any]) -> dict[str, Any]:
    return {key: _to_python(value) for key, value in metadata.items()}


def _iter_step_dicts(tfds, steps: Any):
    if hasattr(steps, "as_numpy_iterator"):
        yield from steps.as_numpy_iterator()
        return

    try:
        yield from tfds.as_numpy(steps)
    except TypeError:
        if isinstance(steps, dict) and "timestamp_ns" in steps:
            timestamps = _to_python(steps["timestamp_ns"])
            if hasattr(timestamps, "tolist"):
                timestamps = timestamps.tolist()
            for index, timestamp_ns in enumerate(timestamps):
                yield {
                    key: _to_python(value)[index]
                    for key, value in steps.items()
                    if key != "observation"
                }


def _step_from_dict(index: int, step: dict[str, Any]) -> RldsStep:
    instruction = step.get("language_instruction")
    if instruction is not None:
        instruction = str(_to_python(instruction))

    return RldsStep(
        index=index,
        timestamp_ns=int(_to_python(step.get("timestamp_ns", index))),
        action=_array_to_float_list(step.get("action")),
        reward=_to_float(step.get("reward")),
        is_first=_to_bool(step.get("is_first")),
        is_last=_to_bool(step.get("is_last")),
        is_terminal=_to_bool(step.get("is_terminal")),
        language_instruction=instruction,
    )


def _sample_indices(count: int, limit: int) -> set[int]:
    if count <= 0 or limit <= 0:
        return set()
    if count <= limit:
        return set(range(count))

    return {
        round(position * (count - 1) / (limit - 1))
        for position in range(limit)
    }


def _encode_png_base64(tf, image: Any) -> str | None:
    try:
        tensor = tf.convert_to_tensor(image)
        if tensor.dtype != tf.uint8:
            tensor = tf.image.convert_image_dtype(tensor, tf.uint8, saturate=True)
        encoded = tf.io.encode_png(tensor).numpy()
    except Exception:
        return None
    return base64.b64encode(encoded).decode("ascii")


def _read_thumbnails(
    *,
    tf,
    tfds,
    raw_steps: Any,
    indices: set[int],
) -> list[RldsThumbnail]:
    thumbnails: list[RldsThumbnail] = []
    if not indices:
        return thumbnails

    for index, step in enumerate(_iter_step_dicts(tfds, raw_steps)):
        if index not in indices:
            continue
        observation = step.get("observation", {})
        image = observation.get("image") if isinstance(observation, dict) else None
        if image is None:
            continue
        png_base64 = _encode_png_base64(tf, image)
        if png_base64 is None:
            continue
        timestamp_ns = int(_to_python(step.get("timestamp_ns", index)))
        thumbnails.append(
            RldsThumbnail(
                step_index=index,
                timestamp_ns=timestamp_ns,
                png_base64=png_base64,
            )
        )

    return thumbnails


def _read_episode_steps(tfds, raw_steps: Any) -> list[RldsStep]:
    steps = []
    for index, step in enumerate(_iter_step_dicts(tfds, raw_steps)):
        steps.append(_step_from_dict(index, step))
    return steps


def _episode_matches(
    metadata: dict[str, Any],
    *,
    bag_path: Path,
    episode_id: str | None,
) -> bool:
    meta_episode_id = str(metadata.get("episode_id", ""))
    meta_file_path = str(metadata.get("file_path", ""))
    if episode_id is not None:
        return meta_episode_id == episode_id

    bag_episode_id = _episode_id_from_path(bag_path)
    if meta_episode_id == bag_episode_id:
        return True
    if meta_file_path:
        meta_path = Path(meta_file_path)
        return _episode_id_from_path(meta_path) == bag_episode_id
    return False


def _load_rlds_episode(
    *,
    rlds_path: Path,
    bag_path: Path,
    dataset_name: str,
    split: str,
    episode_id: str | None,
    episode_index: int | None,
    thumbnail_limit: int,
) -> RldsEpisode:
    tf, tfds = _import_tfds_runtime()
    builder_dir = _resolve_tfds_builder_dir(rlds_path, dataset_name)
    builder = tfds.builder_from_directory(str(builder_dir))
    dataset = builder.as_dataset(split=split)

    first_raw_episode: Any | None = None
    first_metadata: dict[str, Any] | None = None
    first_index = 0

    for index, raw_episode in enumerate(dataset):
        metadata = _metadata_to_dict(raw_episode.get("episode_metadata", {}))
        if first_raw_episode is None:
            first_raw_episode = raw_episode
            first_metadata = metadata
            first_index = index

        matched_index = episode_index is not None and index == episode_index
        matched_metadata = _episode_matches(
            metadata,
            bag_path=bag_path,
            episode_id=episode_id,
        )
        if matched_index or (episode_index is None and matched_metadata):
            return _materialize_episode(
                tf=tf,
                tfds=tfds,
                raw_episode=raw_episode,
                metadata=metadata,
                builder_dir=builder_dir,
                split=split,
                episode_index=index,
                thumbnail_limit=thumbnail_limit,
                selection_note="matched requested episode",
            )

    if episode_index is not None:
        raise ValueError(f"No RLDS episode found at --episode-index {episode_index}")
    if episode_id is not None:
        raise ValueError(f"No RLDS episode found with --episode-id {episode_id!r}")
    if first_raw_episode is None or first_metadata is None:
        raise ValueError(f"RLDS split {split!r} is empty in {builder_dir}")

    return _materialize_episode(
        tf=tf,
        tfds=tfds,
        raw_episode=first_raw_episode,
        metadata=first_metadata,
        builder_dir=builder_dir,
        split=split,
        episode_index=first_index,
        thumbnail_limit=thumbnail_limit,
        selection_note=(
            "no metadata match for the bag path; selected the first RLDS episode"
        ),
    )


def _materialize_episode(
    *,
    tf,
    tfds,
    raw_episode: dict[str, Any],
    metadata: dict[str, Any],
    builder_dir: Path,
    split: str,
    episode_index: int,
    thumbnail_limit: int,
    selection_note: str,
) -> RldsEpisode:
    raw_steps = raw_episode["steps"]
    steps = _read_episode_steps(tfds, raw_steps)
    thumbnail_indices = _sample_indices(len(steps), thumbnail_limit)
    thumbnails = _read_thumbnails(
        tf=tf,
        tfds=tfds,
        raw_steps=raw_steps,
        indices=thumbnail_indices,
    )

    episode_id = str(metadata.get("episode_id", f"episode_{episode_index}"))
    file_path = str(metadata.get("file_path", ""))

    return RldsEpisode(
        builder_dir=builder_dir,
        split=split,
        episode_index=episode_index,
        episode_id=episode_id,
        file_path=file_path,
        metadata=metadata,
        steps=steps,
        thumbnails=thumbnails,
        selection_note=selection_note,
    )


def _nearest_delta_ms(reference_ns: Sequence[int], queries_ns: Sequence[int]):
    if not reference_ns:
        return [None for _ in queries_ns]

    deltas: list[float | None] = []
    for query_ns in queries_ns:
        insert_at = bisect.bisect_left(reference_ns, query_ns)
        candidates = []
        if insert_at > 0:
            candidates.append(reference_ns[insert_at - 1])
        if insert_at < len(reference_ns):
            candidates.append(reference_ns[insert_at])
        if not candidates:
            deltas.append(None)
            continue
        best = min(candidates, key=lambda item: abs(item - query_ns))
        deltas.append((best - query_ns) / 1e6)
    return deltas


def _interval_counts(times_ns: Sequence[int], starts_ns: Sequence[int]) -> list[int]:
    counts = []
    if not starts_ns:
        return counts

    for index, start_ns in enumerate(starts_ns):
        if index + 1 < len(starts_ns):
            end_ns = starts_ns[index + 1]
        elif index > 0:
            end_ns = start_ns + (start_ns - starts_ns[index - 1])
        else:
            end_ns = start_ns
        left = bisect.bisect_left(times_ns, start_ns)
        right = bisect.bisect_left(times_ns, end_ns)
        counts.append(max(right - left, 0))
    return counts


def _series_stats(times_ns: Sequence[int]) -> dict[str, Any]:
    if not times_ns:
        return {
            "count": 0,
            "start_s": None,
            "end_s": None,
            "duration_s": None,
            "rate_hz": None,
        }

    duration_s = (times_ns[-1] - times_ns[0]) / 1e9
    rate_hz = None
    if duration_s > 0 and len(times_ns) > 1:
        rate_hz = (len(times_ns) - 1) / duration_s

    return {
        "count": len(times_ns),
        "start_s": times_ns[0] / 1e9,
        "end_s": times_ns[-1] / 1e9,
        "duration_s": duration_s,
        "rate_hz": rate_hz,
    }


def _numeric_stats(values: Sequence[float | None]) -> dict[str, float | int | None]:
    finite = [abs(value) for value in values if value is not None and math.isfinite(value)]
    if not finite:
        return {"count": 0, "median": None, "p95": None, "max": None}

    sorted_values = sorted(finite)
    p95_index = min(round(0.95 * (len(sorted_values) - 1)), len(sorted_values) - 1)
    return {
        "count": len(sorted_values),
        "median": statistics.median(sorted_values),
        "p95": sorted_values[p95_index],
        "max": max(sorted_values),
    }


def _raw_numeric_stats(values: Sequence[float]) -> dict[str, float | int | None]:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return {"count": 0, "min": None, "median": None, "max": None}
    return {
        "count": len(finite),
        "min": min(finite),
        "median": statistics.median(finite),
        "max": max(finite),
    }


def _gripper_action_analysis(episode: RldsEpisode) -> dict[str, Any]:
    values = [
        step.action[-1]
        for step in episode.steps
        if step.action and math.isfinite(step.action[-1])
    ]
    stats = _raw_numeric_stats(values)
    closed_count = sum(value <= 0.05 for value in values)
    open_count = sum(value >= 0.95 for value in values)
    intermediate_count = len(values) - closed_count - open_count
    rounded_unique = sorted({round(value, 3) for value in values})
    mode = str(episode.metadata.get("gripper_action_mode", "missing"))
    convention = str(episode.metadata.get("gripper_action_convention", "missing"))
    first_count = sum(step.is_first is True for step in episode.steps)
    last_count = sum(step.is_last is True for step in episode.steps)
    terminal_count = sum(step.is_terminal is True for step in episode.steps)

    rows = [
        ["metadata mode", mode],
        ["metadata convention", convention],
        ["value count", stats["count"]],
        ["min", _format_number(stats["min"], digits=4)],
        ["median", _format_number(stats["median"], digits=4)],
        ["max", _format_number(stats["max"], digits=4)],
        ["closed values (<= 0.05)", closed_count],
        ["open values (>= 0.95)", open_count],
        ["intermediate values", intermediate_count],
        ["rounded unique values", rounded_unique[:24]],
        ["is_first count", first_count],
        ["is_last count", last_count],
        ["is_terminal count", terminal_count],
    ]

    warnings = []
    if not values:
        warnings.append("No gripper action values were found in RLDS actions.")
    else:
        min_value = float(stats["min"])
        max_value = float(stats["max"])
        if min_value < -1e-6 or max_value > 1.0 + 1e-6:
            warnings.append(
                "Gripper action values are outside [0, 1]; this looks like "
                "legacy delta or physical-width data, not OpenVLA-OFT absolute "
                "gripper actions."
            )
        if mode in {"missing", "delta", "absolute_width"}:
            warnings.append(
                "RLDS metadata does not declare absolute_binary gripper mode. "
                "Regenerate with --gripper-action-mode absolute_binary before "
                "OpenVLA-OFT fine-tuning."
            )
        if mode == "absolute_binary" and intermediate_count:
            warnings.append(
                "Dataset declares absolute_binary gripper mode but contains "
                "intermediate gripper values; expected only 0.0 and 1.0."
            )
    if first_count != 1:
        warnings.append(
            f"Expected exactly one RLDS step with is_first=True; found {first_count}."
        )
    if last_count != 1:
        warnings.append(
            f"Expected exactly one RLDS step with is_last=True; found {last_count}."
        )
    if terminal_count != 1:
        warnings.append(
            "Expected exactly one RLDS step with is_terminal=True; "
            f"found {terminal_count}."
        )

    return {
        "values": values,
        "rows": rows,
        "warnings": warnings,
        "mode": mode,
        "convention": convention,
        "closed_count": closed_count,
        "open_count": open_count,
        "intermediate_count": intermediate_count,
        "stats": stats,
    }


def _language_instruction_analysis(
    episode: RldsEpisode,
    *,
    instruction_topic: str,
    instruction_times: Sequence[int],
) -> dict[str, Any]:
    instructions = [
        (step.language_instruction or "").strip()
        for step in episode.steps
    ]
    nonempty_count = sum(1 for instruction in instructions if instruction)
    unique_nonempty = sorted({instruction for instruction in instructions if instruction})
    first_step_instruction = instructions[0] if instructions else ""

    rows = [
        ["rosbag instruction topic", instruction_topic],
        ["rosbag instruction messages", len(instruction_times)],
        ["RLDS steps", len(instructions)],
        [
            "RLDS steps with nonempty instruction",
            f"{nonempty_count}/{len(instructions)}",
        ],
        ["first RLDS step instruction", first_step_instruction],
        ["unique nonempty RLDS instructions", unique_nonempty[:12]],
    ]

    warnings = []
    if not instruction_times:
        warnings.append(
            f"No messages found on instruction topic {instruction_topic!r} in the "
            "rosbag being compared. If this is the trimmed bag, the converter "
            "must use --instruction or a preserved pre-window instruction."
        )
    if instructions and not first_step_instruction:
        warnings.append("The first RLDS step has an empty language_instruction.")
    if instructions and nonempty_count != len(instructions):
        warnings.append(
            "RLDS language_instruction is missing on "
            f"{len(instructions) - nonempty_count}/{len(instructions)} step(s)."
        )
    if len(unique_nonempty) > 1:
        warnings.append(
            "RLDS contains multiple distinct language_instruction values in one episode."
        )

    return {
        "rows": rows,
        "warnings": warnings,
        "step_count": len(instructions),
        "nonempty_count": nonempty_count,
        "first_step_instruction": first_step_instruction,
        "unique_nonempty": unique_nonempty,
        "rosbag_instruction_messages": len(instruction_times),
    }


def _format_number(value: Any, digits: int = 3) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, int):
        return str(value)
    try:
        value = float(value)
    except (TypeError, ValueError):
        return html.escape(str(value))
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.{digits}f}"


def _fmt_ms(value: Any) -> str:
    return _format_number(value, digits=3)


def _html_table(headers: Sequence[str], rows: Sequence[Sequence[Any]]) -> str:
    head = "".join(f"<th>{html.escape(header)}</th>" for header in headers)
    body_rows = []
    for row in rows:
        cells = "".join(f"<td>{html.escape(str(cell))}</td>" for cell in row)
        body_rows.append(f"<tr>{cells}</tr>")
    return f"<table><thead><tr>{head}</tr></thead><tbody>{''.join(body_rows)}</tbody></table>"


def _svg_text(x: float, y: float, text: str, size: int = 12) -> str:
    return (
        f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" '
        f'fill="#111827">{html.escape(text)}</text>'
    )


def _decimate_pairs(
    values: Sequence[float | None],
    max_points: int = 1600,
) -> list[tuple[int, float | None]]:
    if len(values) <= max_points:
        return list(enumerate(values))
    stride = math.ceil(len(values) / max_points)
    return [(index, values[index]) for index in range(0, len(values), stride)]


def _svg_line_plot(
    series: Sequence[tuple[str, Sequence[float | None]]],
    *,
    title: str,
    y_label: str,
    height: int = 310,
) -> str:
    width = 1120
    left = 64
    right = 18
    top = 30
    bottom = 42
    plot_w = width - left - right
    plot_h = height - top - bottom

    finite_values = [
        float(value)
        for _label, values in series
        for value in values
        if value is not None and math.isfinite(value)
    ]
    if not finite_values:
        return '<p class="muted">No numeric data available for this plot.</p>'

    x_count = max((len(values) for _label, values in series), default=1)
    y_min = min(finite_values)
    y_max = max(finite_values)
    if math.isclose(y_min, y_max):
        padding = abs(y_min) * 0.1 or 1.0
        y_min -= padding
        y_max += padding
    else:
        padding = (y_max - y_min) * 0.08
        y_min -= padding
        y_max += padding

    def sx(index: int) -> float:
        if x_count <= 1:
            return left
        return left + plot_w * index / (x_count - 1)

    def sy(value: float) -> float:
        return top + plot_h * (y_max - value) / (y_max - y_min)

    elements = [
        f'<svg viewBox="0 0 {width} {height}" role="img" '
        f'aria-label="{html.escape(title)}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        _svg_text(left, 18, title, 14),
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_h}" '
        'stroke="#d1d5db"/>',
        f'<line x1="{left}" y1="{top + plot_h}" x2="{left + plot_w}" '
        f'y2="{top + plot_h}" stroke="#d1d5db"/>',
    ]

    for tick in range(5):
        ratio = tick / 4
        y = top + plot_h * ratio
        value = y_max - (y_max - y_min) * ratio
        elements.append(
            f'<line x1="{left}" y1="{y:.1f}" x2="{left + plot_w}" '
            'y2="{:.1f}" stroke="#f3f4f6"/>'.format(y)
        )
        elements.append(_svg_text(6, y + 4, _format_number(value, 2), 11))

    zero_in_range = y_min < 0 < y_max
    if zero_in_range:
        y = sy(0.0)
        elements.append(
            f'<line x1="{left}" y1="{y:.1f}" x2="{left + plot_w}" '
            'y2="{:.1f}" stroke="#9ca3af" stroke-dasharray="4 4"/>'.format(y)
        )

    for series_index, (label, values) in enumerate(series):
        color = PLOT_COLORS[series_index % len(PLOT_COLORS)]
        points = []
        for index, value in _decimate_pairs(values):
            if value is None or not math.isfinite(value):
                continue
            points.append(f"{sx(index):.1f},{sy(float(value)):.1f}")
        if len(points) >= 2:
            elements.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="1.7" '
                f'points="{" ".join(points)}"/>'
            )
        elif len(points) == 1:
            x, y = points[0].split(",")
            elements.append(
                f'<circle cx="{x}" cy="{y}" r="2.5" fill="{color}"/>'
            )

    legend_x = left
    legend_y = height - 16
    for series_index, (label, _values) in enumerate(series):
        color = PLOT_COLORS[series_index % len(PLOT_COLORS)]
        x = legend_x + series_index * 145
        elements.append(
            f'<rect x="{x}" y="{legend_y - 9}" width="10" height="10" '
            f'fill="{color}"/>'
        )
        elements.append(_svg_text(x + 16, legend_y, label, 11))

    elements.append(_svg_text(width - 102, height - 17, "step index", 11))
    elements.append(_svg_text(7, 18, y_label, 11))
    elements.append("</svg>")
    return "".join(elements)


def _svg_timeline(
    *,
    topic_times_ns: dict[str, list[int]],
    rlds_times_ns: Sequence[int],
) -> str:
    rows = [("RLDS steps", list(rlds_times_ns))]
    for topic, times_ns in topic_times_ns.items():
        rows.append((topic, times_ns))

    all_times = [time_ns for _label, times in rows for time_ns in times]
    if not all_times:
        return '<p class="muted">No timestamps available for timeline.</p>'

    origin = min(all_times)
    end = max(all_times)
    duration = max((end - origin) / 1e9, 1e-9)

    width = 1120
    left = 250
    right = 18
    top = 36
    row_h = 30
    height = top + row_h * len(rows) + 48
    plot_w = width - left - right

    def sx(time_ns: int) -> float:
        return left + plot_w * ((time_ns - origin) / 1e9) / duration

    elements = [
        f'<svg viewBox="0 0 {width} {height}" role="img" '
        'aria-label="rosbag and RLDS timestamp timeline">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        _svg_text(left, 20, "Time alignment", 14),
    ]

    for tick in range(6):
        x = left + plot_w * tick / 5
        label = f"{duration * tick / 5:.2f}s"
        elements.append(
            f'<line x1="{x:.1f}" y1="{top - 10}" x2="{x:.1f}" '
            f'y2="{height - 36}" stroke="#f3f4f6"/>'
        )
        elements.append(_svg_text(x - 12, height - 16, label, 11))

    for row_index, (label, times_ns) in enumerate(rows):
        y = top + row_index * row_h
        color = PLOT_COLORS[row_index % len(PLOT_COLORS)]
        elements.append(_svg_text(10, y + 5, f"{label} ({len(times_ns)})", 11))
        elements.append(
            f'<line x1="{left}" y1="{y}" x2="{left + plot_w}" y2="{y}" '
            'stroke="#e5e7eb"/>'
        )
        if not times_ns:
            continue
        stride = max(math.ceil(len(times_ns) / 1800), 1)
        for time_ns in times_ns[::stride]:
            x = sx(time_ns)
            elements.append(
                f'<line x1="{x:.1f}" y1="{y - 8}" x2="{x:.1f}" '
                f'y2="{y + 8}" stroke="{color}" stroke-width="1.1"/>'
            )

    elements.append("</svg>")
    return "".join(elements)


def _summary_cards(metrics: dict[str, Any]) -> str:
    cards = []
    for label, value in metrics.items():
        cards.append(
            '<div class="metric">'
            f'<div class="metric-label">{html.escape(label)}</div>'
            f'<div class="metric-value">{html.escape(str(value))}</div>'
            '</div>'
        )
    return f'<div class="metrics">{"".join(cards)}</div>'


def _thumbnail_grid(thumbnails: Sequence[RldsThumbnail]) -> str:
    if not thumbnails:
        return '<p class="muted">No RLDS image thumbnails could be rendered.</p>'

    cards = []
    for thumb in thumbnails:
        timestamp_s = thumb.timestamp_ns / 1e9
        cards.append(
            '<figure class="thumb">'
            f'<img src="data:image/png;base64,{thumb.png_base64}" '
            f'alt="RLDS step {thumb.step_index}"/>'
            '<figcaption>'
            f'step {thumb.step_index}<br>{timestamp_s:.3f}s'
            '</figcaption></figure>'
        )
    return f'<div class="thumb-grid">{"".join(cards)}</div>'


def _build_analysis(
    *,
    bag: RosbagSummary,
    episode: RldsEpisode,
    max_sync_dt_sec: float,
    camera_topic: str,
    joint_topic: str,
    robot_state_topic: str,
    action_topic: str,
    instruction_topic: str,
    gripper_state_topic: str,
) -> dict[str, Any]:
    rlds_times = [step.timestamp_ns for step in episode.steps]
    camera_times = bag.topic_times_ns.get(camera_topic, [])
    robot_times = bag.topic_times_ns.get(robot_state_topic, [])
    joint_times = bag.topic_times_ns.get(joint_topic, [])
    action_times = bag.topic_times_ns.get(action_topic, [])
    instruction_times = bag.topic_times_ns.get(instruction_topic, [])
    gripper_state_times = bag.topic_times_ns.get(gripper_state_topic, [])

    camera_candidate_times = camera_times[:-1] if len(camera_times) > 1 else []
    rlds_time_set = set(rlds_times)
    camera_time_set = set(camera_times)

    missing_candidate_times = [
        time_ns for time_ns in camera_candidate_times if time_ns not in rlds_time_set
    ]
    extra_rlds_times = [
        time_ns for time_ns in rlds_times if time_ns not in camera_time_set
    ]
    exact_matches = len(rlds_times) - len(extra_rlds_times)

    deltas = {
        "camera": _nearest_delta_ms(camera_times, rlds_times),
        "robot_state": _nearest_delta_ms(robot_times, rlds_times),
        "joint_state": _nearest_delta_ms(joint_times, rlds_times),
        "action": _nearest_delta_ms(action_times, rlds_times),
        "gripper_state": _nearest_delta_ms(gripper_state_times, rlds_times),
    }
    action_interval_counts = _interval_counts(action_times, rlds_times)
    max_sync_ms = max_sync_dt_sec * 1000.0

    sync_rows = []
    for label, values in deltas.items():
        stats = _numeric_stats(values)
        over_limit = sum(
            1
            for value in values
            if value is None or abs(value) > max_sync_ms
        )
        sync_rows.append(
            [
                label,
                stats["count"],
                _fmt_ms(stats["median"]),
                _fmt_ms(stats["p95"]),
                _fmt_ms(stats["max"]),
                over_limit,
            ]
        )

    topic_rows = []
    topic_order = [
        camera_topic,
        robot_state_topic,
        joint_topic,
        action_topic,
        gripper_state_topic,
        instruction_topic,
    ]
    for topic in dict.fromkeys(topic_order):
        times_ns = bag.topic_times_ns.get(topic, [])
        stats = _series_stats(times_ns)
        topic_rows.append(
            [
                topic,
                bag.topic_types.get(topic, "missing"),
                stats["count"],
                _format_number(stats["duration_s"]),
                _format_number(stats["rate_hz"]),
            ]
        )

    action_rows = []
    if action_interval_counts:
        stats = _numeric_stats([float(value) for value in action_interval_counts])
        action_rows.append(
            [
                "twist samples per RLDS step interval",
                _format_number(stats["median"]),
                _format_number(stats["p95"]),
                _format_number(stats["max"]),
            ]
        )

    gripper_analysis = _gripper_action_analysis(episode)
    language_analysis = _language_instruction_analysis(
        episode,
        instruction_topic=instruction_topic,
        instruction_times=instruction_times,
    )
    warnings = []
    missing_topics = [
        topic
        for topic in dict.fromkeys(topic_order)
        if topic != instruction_topic and not bag.topic_times_ns.get(topic)
    ]
    if missing_topics:
        warnings.append("Missing expected rosbag topic(s): " + ", ".join(missing_topics))
    if extra_rlds_times:
        warnings.append(
            f"{len(extra_rlds_times)} RLDS step timestamp(s) were not exact "
            "camera timestamp matches."
        )
    if missing_candidate_times:
        warnings.append(
            f"{len(missing_candidate_times)} camera frame(s), excluding the final "
            "frame, were not present as RLDS steps."
        )
    warnings.extend(language_analysis["warnings"])
    warnings.extend(gripper_analysis["warnings"])
    if not warnings:
        warnings.append("No timing or OpenVLA gripper issues were detected.")

    return {
        "rlds_times": rlds_times,
        "camera_times": camera_times,
        "robot_times": robot_times,
        "joint_times": joint_times,
        "action_times": action_times,
        "instruction_times": instruction_times,
        "gripper_state_times": gripper_state_times,
        "camera_candidate_count": len(camera_candidate_times),
        "missing_candidate_count": len(missing_candidate_times),
        "extra_rlds_count": len(extra_rlds_times),
        "exact_matches": exact_matches,
        "deltas": deltas,
        "sync_rows": sync_rows,
        "topic_rows": topic_rows,
        "action_interval_counts": action_interval_counts,
        "action_rows": action_rows,
        "gripper_rows": gripper_analysis["rows"],
        "gripper_analysis": gripper_analysis,
        "language_rows": language_analysis["rows"],
        "language_analysis": language_analysis,
        "warnings": warnings,
    }


def _actions_series(steps: Sequence[RldsStep]):
    max_dims = max((len(step.action) for step in steps), default=0)
    series = []
    for dim in range(min(max_dims, len(ACTION_LABELS))):
        values = [
            step.action[dim] if dim < len(step.action) else None
            for step in steps
        ]
        series.append((ACTION_LABELS[dim], values))
    return series


def _render_report(
    *,
    bag_input: Path,
    bag: RosbagSummary,
    episode: RldsEpisode,
    analysis: dict[str, Any],
    output_path: Path,
    max_sync_dt_sec: float,
    camera_topic: str,
    joint_topic: str,
    robot_state_topic: str,
    action_topic: str,
    instruction_topic: str,
    gripper_state_topic: str,
) -> None:
    metrics = {
        "RLDS steps": len(episode.steps),
        "camera frames": len(analysis["camera_times"]),
        "expected camera steps": analysis["camera_candidate_count"],
        "exact timestamp matches": analysis["exact_matches"],
        "missing camera steps": analysis["missing_candidate_count"],
        "extra RLDS steps": analysis["extra_rlds_count"],
    }

    topic_times = {
        camera_topic: analysis["camera_times"],
        robot_state_topic: analysis["robot_times"],
        joint_topic: analysis["joint_times"],
        action_topic: analysis["action_times"],
        gripper_state_topic: analysis["gripper_state_times"],
        instruction_topic: analysis["instruction_times"],
    }

    sync_plot = _svg_line_plot(
        [
            ("camera", analysis["deltas"]["camera"]),
            ("robot", analysis["deltas"]["robot_state"]),
            ("joint", analysis["deltas"]["joint_state"]),
            ("action", analysis["deltas"]["action"]),
            ("gripper", analysis["deltas"]["gripper_state"]),
        ],
        title="Nearest rosbag sample offset from each RLDS step",
        y_label="ms",
    )
    action_plot = _svg_line_plot(
        _actions_series(episode.steps),
        title="RLDS action values",
        y_label="action",
    )
    interval_plot = _svg_line_plot(
        [("twist samples", [float(v) for v in analysis["action_interval_counts"]])],
        title="rosbag action samples inside each RLDS step interval",
        y_label="count",
    )

    warning_items = "".join(
        f"<li>{html.escape(warning)}</li>" for warning in analysis["warnings"]
    )
    first_instruction = next(
        (
            step.language_instruction
            for step in episode.steps
            if step.language_instruction
        ),
        "",
    )

    report = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RLDS rosbag2 correspondence report</title>
<style>
:root {{
  color: #111827;
  background: #f9fafb;
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
}}
body {{
  margin: 0;
  padding: 24px;
}}
main {{
  max-width: 1160px;
  margin: 0 auto;
}}
h1, h2 {{
  margin: 0 0 12px;
}}
h1 {{
  font-size: 28px;
}}
h2 {{
  font-size: 18px;
  margin-top: 28px;
}}
.subtle, .muted {{
  color: #6b7280;
}}
.panel {{
  background: #ffffff;
  border: 1px solid #e5e7eb;
  border-radius: 8px;
  padding: 18px;
  margin: 14px 0;
  overflow-x: auto;
}}
.metrics {{
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
  gap: 10px;
  margin: 16px 0;
}}
.metric {{
  border: 1px solid #e5e7eb;
  border-radius: 8px;
  padding: 12px;
  background: #ffffff;
}}
.metric-label {{
  color: #6b7280;
  font-size: 12px;
}}
.metric-value {{
  font-size: 24px;
  font-weight: 650;
  margin-top: 4px;
}}
table {{
  border-collapse: collapse;
  width: 100%;
  font-size: 13px;
}}
th, td {{
  border-bottom: 1px solid #e5e7eb;
  padding: 8px 10px;
  text-align: left;
  vertical-align: top;
}}
th {{
  color: #374151;
  background: #f3f4f6;
}}
code {{
  background: #f3f4f6;
  border-radius: 4px;
  padding: 2px 4px;
}}
svg {{
  width: 100%;
  height: auto;
}}
.thumb-grid {{
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
  gap: 12px;
}}
.thumb {{
  margin: 0;
  border: 1px solid #e5e7eb;
  border-radius: 8px;
  overflow: hidden;
  background: #ffffff;
}}
.thumb img {{
  width: 100%;
  display: block;
  aspect-ratio: 1 / 1;
  object-fit: cover;
}}
.thumb figcaption {{
  color: #4b5563;
  font-size: 12px;
  padding: 8px;
}}
</style>
</head>
<body>
<main>
<h1>RLDS rosbag2 correspondence report</h1>
<p class="subtle">
Generated {html.escape(datetime.now().isoformat(timespec="seconds"))}.
This report reads rosbag2 sqlite timestamps directly and does not import ROS 2.
</p>
{_summary_cards(metrics)}

<section class="panel">
<h2>Selected inputs</h2>
<table>
<tbody>
<tr><th>rosbag2 input</th><td><code>{html.escape(str(bag_input))}</code></td></tr>
<tr><th>rosbag2 sqlite source</th><td><code>{html.escape(str(bag.bag_uri))}</code></td></tr>
<tr><th>RLDS builder</th><td><code>{html.escape(str(episode.builder_dir))}</code></td></tr>
<tr><th>RLDS split/index</th><td>{html.escape(episode.split)} / {episode.episode_index}</td></tr>
<tr><th>RLDS episode id</th><td>{html.escape(episode.episode_id)}</td></tr>
<tr><th>RLDS file_path</th><td><code>{html.escape(episode.file_path)}</code></td></tr>
<tr><th>Selection note</th><td>{html.escape(episode.selection_note)}</td></tr>
<tr><th>Language instruction</th><td>{html.escape(first_instruction)}</td></tr>
<tr><th>Gripper state topic</th><td><code>{html.escape(str(episode.metadata.get("gripper_state_topic", gripper_state_topic)))}</code></td></tr>
<tr><th>Gripper action mode</th><td>{html.escape(str(episode.metadata.get("gripper_action_mode", "missing")))}</td></tr>
<tr><th>Gripper convention</th><td>{html.escape(str(episode.metadata.get("gripper_action_convention", "missing")))}</td></tr>
<tr><th>Sync threshold</th><td>{max_sync_dt_sec:.3f}s</td></tr>
</tbody>
</table>
</section>

<section class="panel">
<h2>Findings</h2>
<ul>{warning_items}</ul>
</section>

<section class="panel">
{_svg_timeline(topic_times_ns=topic_times, rlds_times_ns=analysis["rlds_times"])}
</section>

<section class="panel">
<h2>Topic timing</h2>
{_html_table(
    ["topic", "type", "count", "duration s", "rate Hz"],
    analysis["topic_rows"],
)}
</section>

<section class="panel">
<h2>Language instruction coverage</h2>
{_html_table(
    ["metric", "value"],
    analysis["language_rows"],
)}
</section>

<section class="panel">
<h2>Nearest sample offsets</h2>
<p class="subtle">
Values are absolute nearest rosbag timestamp offsets from each RLDS step, in ms.
The converter writes RLDS timestamps from camera frame timestamps, so camera
offsets should normally be exactly 0 ms.
</p>
{_html_table(
    ["stream", "matched", "median ms", "p95 ms", "max ms", "over threshold"],
    analysis["sync_rows"],
)}
{sync_plot}
</section>

<section class="panel">
<h2>Action coverage</h2>
{_html_table(
    ["metric", "median", "p95", "max"],
    analysis["action_rows"],
)}
{interval_plot}
{action_plot}
</section>

<section class="panel">
<h2>OpenVLA gripper check</h2>
{_html_table(
    ["metric", "value"],
    analysis["gripper_rows"],
)}
</section>

<section class="panel">
<h2>RLDS image preview</h2>
{_thumbnail_grid(episode.thumbnails)}
</section>
</main>
</body>
</html>
"""

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(report, encoding="utf-8")


def _write_summary_json(
    *,
    output_path: Path,
    bag: RosbagSummary,
    episode: RldsEpisode,
    analysis: dict[str, Any],
) -> None:
    payload = {
        "bag_uri": str(bag.bag_uri),
        "db3_files": [str(path) for path in bag.db3_files],
        "rlds_builder_dir": str(episode.builder_dir),
        "episode_index": episode.episode_index,
        "episode_id": episode.episode_id,
        "episode_metadata": episode.metadata,
        "rlds_steps": len(episode.steps),
        "camera_frames": len(analysis["camera_times"]),
        "camera_candidate_count": analysis["camera_candidate_count"],
        "exact_matches": analysis["exact_matches"],
        "missing_candidate_count": analysis["missing_candidate_count"],
        "extra_rlds_count": analysis["extra_rlds_count"],
        "gripper_analysis": {
            key: value
            for key, value in analysis["gripper_analysis"].items()
            if key != "values"
        },
        "language_analysis": analysis["language_analysis"],
        "warnings": analysis["warnings"],
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Create a ROS2-free HTML report comparing rosbag2 timestamps with "
            "a converted RLDS/TFDS episode."
        )
    )
    parser.add_argument(
        "bag",
        type=Path,
        help="rosbag2 directory, .db3 file, or zipped rosbag2 directory.",
    )
    parser.add_argument(
        "--rlds",
        required=True,
        type=Path,
        help=(
            "TFDS builder directory, e.g. ~/tensorflow_datasets/franka_vla/1.0.0. "
            "A data root containing franka_vla/1.0.0 also works."
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("rlds_rosbag_report.html"),
        help="HTML report path.",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help="Optional machine-readable summary path.",
    )
    parser.add_argument("--dataset-name", default="franka_vla")
    parser.add_argument("--split", default="train")
    parser.add_argument("--episode-id", default=None)
    parser.add_argument("--episode-index", type=int, default=None)
    parser.add_argument("--camera-topic", default=DEFAULT_CAMERA_TOPIC)
    parser.add_argument("--joint-topic", default=DEFAULT_JOINT_TOPIC)
    parser.add_argument("--robot-state-topic", default=DEFAULT_ROBOT_STATE_TOPIC)
    parser.add_argument("--action-topic", default=DEFAULT_ACTION_TOPIC)
    parser.add_argument("--instruction-topic", default=DEFAULT_INSTRUCTION_TOPIC)
    parser.add_argument(
        "--gripper-state-topic",
        default=DEFAULT_GRIPPER_STATE_TOPIC,
        help=(
            "Topic used by rosbag2_to_rlds to infer OpenVLA-OFT gripper "
            "actions. Defaults to --joint-topic for the current recorder."
        ),
    )
    parser.add_argument(
        "--max-sync-dt-sec",
        type=float,
        default=0.10,
        help="Threshold used for highlighting timing offsets.",
    )
    parser.add_argument(
        "--max-thumbnails",
        type=int,
        default=12,
        help="Maximum evenly sampled RLDS image thumbnails to embed.",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
    args = _parse_args(argv)
    topics = [
        args.camera_topic,
        args.joint_topic,
        args.robot_state_topic,
        args.action_topic,
        args.gripper_state_topic,
        args.instruction_topic,
    ]
    topics = list(dict.fromkeys(topics))

    with tempfile.TemporaryDirectory(prefix="rlds_rosbag_report_") as temp_name:
        bag_uri = _bag_uri_from_input(args.bag, Path(temp_name))
        bag = _read_rosbag2_sqlite(bag_uri, topics)

        episode = _load_rlds_episode(
            rlds_path=args.rlds,
            bag_path=args.bag,
            dataset_name=args.dataset_name,
            split=args.split,
            episode_id=args.episode_id,
            episode_index=args.episode_index,
            thumbnail_limit=max(args.max_thumbnails, 0),
        )

        analysis = _build_analysis(
            bag=bag,
            episode=episode,
            max_sync_dt_sec=args.max_sync_dt_sec,
            camera_topic=args.camera_topic,
            joint_topic=args.joint_topic,
            robot_state_topic=args.robot_state_topic,
            action_topic=args.action_topic,
            instruction_topic=args.instruction_topic,
            gripper_state_topic=args.gripper_state_topic,
        )

        _render_report(
            bag_input=args.bag,
            bag=bag,
            episode=episode,
            analysis=analysis,
            output_path=args.output,
            max_sync_dt_sec=args.max_sync_dt_sec,
            camera_topic=args.camera_topic,
            joint_topic=args.joint_topic,
            robot_state_topic=args.robot_state_topic,
            action_topic=args.action_topic,
            instruction_topic=args.instruction_topic,
            gripper_state_topic=args.gripper_state_topic,
        )

        if args.summary_json is not None:
            _write_summary_json(
                output_path=args.summary_json,
                bag=bag,
                episode=episode,
                analysis=analysis,
            )

    print(f"Wrote report to: {args.output.expanduser().resolve()}")
    if args.summary_json is not None:
        print(f"Wrote summary to: {args.summary_json.expanduser().resolve()}")


if __name__ == "__main__":
    main()
