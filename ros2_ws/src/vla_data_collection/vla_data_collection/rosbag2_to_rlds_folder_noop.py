"""Batch-convert rosbag2 episodes to RLDS while optionally omitting no-op steps.

This standalone command discovers bags, groups instructions, converts episodes,
and optionally filters the synchronized RLDS steps produced by
``rosbag2_to_rlds``. It never deletes messages from the source rosbag2 files.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence

import numpy as np

from vla_data_collection.rosbag2_to_rlds import (
    DEFAULT_ACTION_TOPIC,
    DEFAULT_CAMERA_TOPIC,
    DEFAULT_GRIPPER_BINARY_MOTION_THRESHOLD_M,
    DEFAULT_GRIPPER_STATE_TOPIC,
    DEFAULT_INSTRUCTION_TOPIC,
    DEFAULT_JOINT_TOPIC,
    DEFAULT_MAX_GRIPPER_WIDTH_M,
    DEFAULT_ROBOT_STATE_TOPIC,
    ConverterConfig,
    _finalize_step_flags,
    convert_bags,
)


DEFAULT_IMAGE_SIZE = 256
DEFAULT_LINEAR_NOOP_THRESHOLD_M = 0.0005
DEFAULT_ANGULAR_NOOP_THRESHOLD_RAD = 0.005
DEFAULT_DATASET_NAME = "franka_vla"
REPORT_FILE_NAME = "rlds_noop_filter_report.pdf"


@dataclass(frozen=True)
class BagPlan:
    path: Path
    instruction: str | None
    group_name: str | None


@dataclass(frozen=True)
class NoopFilterStats:
    input_steps: int
    output_steps: int
    omitted_steps: int
    preserved_gripper_transitions: int


@dataclass(frozen=True)
class EpisodeStepCount:
    episode_id: str
    file_path: str
    steps: int


def _is_rosbag_dir(path: Path) -> bool:
    return path.is_dir() and (path / "metadata.yaml").is_file()


def _is_rosbag_file(path: Path) -> bool:
    return path.is_file() and path.suffix.lower() in {".db3", ".zip"}


def _pretty_instruction_from_folder(path: Path) -> str:
    return path.name.replace("_", " ").replace("-", " ").strip() or path.name


def _read_group_instruction(group_dir: Path, instruction_file_name: str) -> str:
    instruction_file = group_dir / instruction_file_name
    if instruction_file.is_file():
        instruction = instruction_file.read_text(encoding="utf-8").strip()
        if not instruction:
            raise ValueError(f"Instruction file is empty: {instruction_file}")
        return instruction

    return _pretty_instruction_from_folder(group_dir)


def _find_bags_recursive(root: Path) -> list[Path]:
    if _is_rosbag_dir(root):
        return [root]

    bags: list[Path] = []
    for child in sorted(root.iterdir(), key=lambda p: str(p)):
        if _is_rosbag_file(child):
            bags.append(child)
        elif child.is_dir():
            bags.extend(_find_bags_recursive(child))
    return bags


def discover_bags(input_dir: Path, instruction_file_name: str) -> list[BagPlan]:
    root = input_dir.expanduser().resolve()
    if not root.is_dir():
        raise ValueError(f"Input path is not a directory: {root}")

    if _is_rosbag_dir(root):
        return [BagPlan(path=root, instruction=None, group_name=None)]

    plans: list[BagPlan] = []
    for child in sorted(root.iterdir(), key=lambda p: str(p)):
        if child.name == instruction_file_name:
            continue

        if _is_rosbag_file(child) or _is_rosbag_dir(child):
            plans.append(BagPlan(path=child, instruction=None, group_name=None))
            continue

        if not child.is_dir():
            continue

        group_bags = _find_bags_recursive(child)
        if not group_bags:
            continue

        instruction = _read_group_instruction(child, instruction_file_name)
        plans.extend(
            BagPlan(path=bag, instruction=instruction, group_name=child.name)
            for bag in group_bags
        )

    return sorted(plans, key=lambda plan: str(plan.path))


def _decode_text(value: object) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return str(value)


def omit_near_zero_actions(
    episode: dict,
    *,
    linear_threshold_m: float,
    angular_threshold_rad: float,
) -> NoopFilterStats:
    """Drop stationary arm actions without ever dropping a gripper transition.

    A step is omitted only when its linear and angular action norms are both
    below the supplied thresholds *and* its binary gripper command is unchanged
    from the preceding raw step.  The first and final step are retained so an
    episode continues to include an initial observation and a terminal sample.
    """

    if linear_threshold_m < 0.0:
        raise ValueError("--linear-noop-threshold-m must be non-negative")
    if angular_threshold_rad < 0.0:
        raise ValueError("--angular-noop-threshold-rad must be non-negative")

    steps = episode["steps"]
    if not steps:
        raise ValueError("Cannot filter an episode with zero RLDS steps")

    kept_steps: list[dict] = []
    previous_raw_gripper: float | None = None
    preserved_gripper_transitions = 0
    last_index = len(steps) - 1

    for index, step in enumerate(steps):
        action = np.asarray(step["action"], dtype=np.float64)
        if action.shape != (7,):
            raise ValueError(
                "Near-zero filtering expects a 7-D action "
                "[dx, dy, dz, droll, dpitch, dyaw, gripper]"
            )

        gripper = float(action[6])
        gripper_changed = (
            previous_raw_gripper is not None and gripper != previous_raw_gripper
        )
        linear_is_small = float(np.linalg.norm(action[:3])) < linear_threshold_m
        angular_is_small = float(np.linalg.norm(action[3:6])) < angular_threshold_rad
        is_noop = linear_is_small and angular_is_small and not gripper_changed

        keep_step = index == 0 or index == last_index or not is_noop
        if keep_step:
            kept_steps.append(step)
            if gripper_changed:
                preserved_gripper_transitions += 1

        # Compare with the preceding *raw* label, even when that label was
        # omitted.  This guarantees every 0->1 or 1->0 gripper transition is kept.
        previous_raw_gripper = gripper

    episode["steps"] = kept_steps
    _finalize_step_flags(
        kept_steps,
        success=bool(episode["episode_metadata"]["success"]),
    )
    return NoopFilterStats(
        input_steps=len(steps),
        output_steps=len(kept_steps),
        omitted_steps=len(steps) - len(kept_steps),
        preserved_gripper_transitions=preserved_gripper_transitions,
    )


def _print_plan(
    plans: Sequence[BagPlan],
    *,
    fallback_instruction: str | None,
    show_bags: bool,
) -> None:
    grouped: dict[str, list[Path]] = defaultdict(list)
    for plan in plans:
        instruction = (
            plan.instruction
            or fallback_instruction
            or "<from recorded instruction topic or episode name>"
        )
        grouped[instruction].append(plan.path)

    print(f"Discovered {len(plans)} rosbag2 bag(s) in {len(grouped)} instruction group(s).")
    for instruction in sorted(grouped):
        bags = grouped[instruction]
        print(f"\nInstruction: {instruction}")
        print(f"  Bags: {len(bags)}")
        if show_bags:
            for bag in bags:
                print(f"    {bag}")


def _instruction_overrides(plans: Sequence[BagPlan]) -> dict[str, str]:
    overrides: dict[str, str] = {}
    for plan in plans:
        if plan.instruction:
            overrides[str(plan.path.expanduser().resolve())] = plan.instruction
    return overrides


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Discover rosbag2 episodes under a folder, convert them into one "
            "RLDS/TFDS dataset, and omit stationary near-zero actions while "
            "preserving every binary gripper transition."
        )
    )
    parser.add_argument(
        "input_dir",
        type=Path,
        help="Folder containing rosbag2 bags, or instruction subfolders containing bags.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="TFDS data root. The dataset will be written below this directory.",
    )
    parser.add_argument(
        "--dataset-name",
        default=None,
        help="TFDS dataset name, for example panda_vla_no_noops. Defaults to franka_vla.",
    )
    parser.add_argument(
        "--instruction",
        default=None,
        help=(
            "Fallback instruction for bags found directly inside input_dir. "
            "Instruction subfolders still use their folder instruction."
        ),
    )
    parser.add_argument(
        "--instruction-file-name",
        default="instruction.txt",
        help=(
            "Optional file name inside an instruction subfolder. If present, its "
            "contents are used instead of the folder name."
        ),
    )
    parser.add_argument("--camera-topic", default=DEFAULT_CAMERA_TOPIC)
    parser.add_argument("--joint-topic", default=DEFAULT_JOINT_TOPIC)
    parser.add_argument("--robot-state-topic", default=DEFAULT_ROBOT_STATE_TOPIC)
    parser.add_argument("--action-topic", default=DEFAULT_ACTION_TOPIC)
    parser.add_argument("--instruction-topic", default=DEFAULT_INSTRUCTION_TOPIC)
    parser.add_argument("--gripper-state-topic", default=DEFAULT_GRIPPER_STATE_TOPIC)
    parser.add_argument("--image-size", type=int, default=DEFAULT_IMAGE_SIZE)
    parser.add_argument(
        "--resize-mode",
        choices=("center_crop", "stretch"),
        default="center_crop",
    )
    parser.add_argument("--max-sync-dt-sec", type=float, default=0.10)
    parser.add_argument(
        "--twist-action-mode",
        choices=("integrated_delta", "raw_twist"),
        default="integrated_delta",
    )
    parser.add_argument(
        "--gripper-action-mode",
        choices=("absolute_binary", "absolute_width", "delta"),
        default="absolute_binary",
    )
    parser.add_argument(
        "--max-gripper-width-m",
        type=float,
        default=DEFAULT_MAX_GRIPPER_WIDTH_M,
    )
    parser.add_argument(
        "--gripper-binary-motion-threshold-m",
        type=float,
        default=DEFAULT_GRIPPER_BINARY_MOTION_THRESHOLD_M,
    )
    parser.add_argument("--embodiment", default="franka_panda")
    parser.add_argument(
        "--success",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Whether to mark converted demonstrations as successful.",
    )
    parser.add_argument(
        "--keep-near-zero-actions",
        action="store_true",
        help=(
            "Disable no-op omission and retain every synchronized RLDS step. "
            "This is useful for an unfiltered comparison dataset."
        ),
    )
    parser.add_argument(
        "--linear-noop-threshold-m",
        type=float,
        default=DEFAULT_LINEAR_NOOP_THRESHOLD_M,
        help=(
            "Omit a stationary step only when its translation norm is below this "
            f"value. Default: {DEFAULT_LINEAR_NOOP_THRESHOLD_M} m."
        ),
    )
    parser.add_argument(
        "--angular-noop-threshold-rad",
        type=float,
        default=DEFAULT_ANGULAR_NOOP_THRESHOLD_RAD,
        help=(
            "Omit a stationary step only when its rotation norm is below this "
            f"value. Default: {DEFAULT_ANGULAR_NOOP_THRESHOLD_RAD} rad."
        ),
    )
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print discovered bags and instructions without converting.",
    )
    return parser.parse_args(argv)


def _validate_args(args: argparse.Namespace) -> None:
    if args.linear_noop_threshold_m < 0.0:
        raise SystemExit("--linear-noop-threshold-m must be non-negative")
    if args.angular_noop_threshold_rad < 0.0:
        raise SystemExit("--angular-noop-threshold-rad must be non-negative")
    if args.keep_near_zero_actions:
        return
    if args.twist_action_mode != "integrated_delta":
        raise SystemExit(
            "Near-zero thresholds are measured in m and rad, so filtering requires "
            "--twist-action-mode integrated_delta. Use --keep-near-zero-actions "
            "with raw_twist."
        )
    if args.gripper_action_mode != "absolute_binary":
        raise SystemExit(
            "Near-zero filtering requires --gripper-action-mode absolute_binary "
            "to guarantee that every open/close transition is retained. Use "
            "--keep-near-zero-actions with another gripper action mode."
        )


def _make_config(
    args: argparse.Namespace,
    instruction_overrides: dict[str, str],
) -> ConverterConfig:
    return ConverterConfig(
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


def _read_episode_step_counts(data_dir: Path) -> list[EpisodeStepCount]:
    """Read the finished TFDS dataset so report counts match the on-disk RLDS."""

    try:
        import tensorflow_datasets as tfds  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "Cannot create the RLDS count report because tensorflow_datasets is "
            "missing. Install requirements-rlds.txt after sourcing ROS."
        ) from exc

    builder = tfds.builder_from_directory(str(data_dir))
    dataset = builder.as_dataset(split="train", shuffle_files=False)
    counts: list[EpisodeStepCount] = []
    for episode in dataset:
        metadata = episode["episode_metadata"]
        counts.append(
            EpisodeStepCount(
                episode_id=_decode_text(metadata["episode_id"].numpy()),
                file_path=_decode_text(metadata["file_path"].numpy()),
                steps=sum(1 for _ in episode["steps"]),
            )
        )
    return counts


def _write_pdf_report(
    *,
    data_dir: Path,
    episode_counts: Sequence[EpisodeStepCount],
    filtering_enabled: bool,
    linear_threshold_m: float,
    angular_threshold_rad: float,
) -> Path:
    """Write a paginated report next to the generated RLDS dataset."""

    try:
        from reportlab.lib import colors  # type: ignore
        from reportlab.lib.enums import TA_CENTER  # type: ignore
        from reportlab.lib.pagesizes import A4  # type: ignore
        from reportlab.lib.styles import getSampleStyleSheet  # type: ignore
        from reportlab.lib.units import mm  # type: ignore
        from reportlab.platypus import (  # type: ignore
            Paragraph,
            SimpleDocTemplate,
            Spacer,
            Table,
            TableStyle,
        )
    except ImportError as exc:
        raise SystemExit(
            "Cannot create the PDF report because reportlab is missing. Install "
            "requirements-rlds.txt after sourcing ROS."
        ) from exc

    report_path = data_dir / REPORT_FILE_NAME
    total_steps = sum(count.steps for count in episode_counts)
    styles = getSampleStyleSheet()
    title_style = styles["Title"].clone("NoopReportTitle")
    title_style.alignment = TA_CENTER
    heading_style = styles["Heading2"]
    normal_style = styles["BodyText"]
    small_style = styles["BodyText"].clone("NoopReportSmall")
    small_style.fontSize = 7.5
    small_style.leading = 9

    story = [
        Paragraph("RLDS Near-Zero Action Conversion Report", title_style),
        Spacer(1, 5 * mm),
        Paragraph("Dataset summary", heading_style),
        Paragraph(f"Dataset directory: {data_dir}", normal_style),
        Paragraph(
            "Generated: "
            + datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC"),
            normal_style,
        ),
        Paragraph(
            "No-op omission: " + ("enabled" if filtering_enabled else "disabled"),
            normal_style,
        ),
    ]
    if filtering_enabled:
        story.extend(
            [
                Paragraph(
                    "A step was omitted only when both arm-action norms were below "
                    "the thresholds and its binary gripper command was unchanged. "
                    "First, final, and every 0-to-1 or 1-to-0 gripper-transition "
                    "step were retained.",
                    normal_style,
                ),
                Paragraph(
                    f"Translation threshold: {linear_threshold_m:.7g} m; "
                    f"rotation threshold: {angular_threshold_rad:.7g} rad.",
                    normal_style,
                ),
            ]
        )
    story.extend(
        [
            Spacer(1, 4 * mm),
            Table(
                [
                    ["Total episodes", str(len(episode_counts))],
                    ["Total RLDS steps", str(total_steps)],
                ],
                colWidths=[55 * mm, 45 * mm],
                style=TableStyle(
                    [
                        ("BACKGROUND", (0, 0), (0, -1), colors.HexColor("#E8EEF7")),
                        ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#8394AA")),
                        ("FONTNAME", (0, 0), (0, -1), "Helvetica-Bold"),
                        ("ALIGN", (1, 0), (1, -1), "RIGHT"),
                        ("LEFTPADDING", (0, 0), (-1, -1), 7),
                        ("RIGHTPADDING", (0, 0), (-1, -1), 7),
                        ("TOPPADDING", (0, 0), (-1, -1), 5),
                        ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
                    ]
                ),
            ),
            Spacer(1, 7 * mm),
            Paragraph("Episode step counts", heading_style),
        ]
    )

    rows = [["#", "Episode ID", "Steps", "Source bag"]]
    for index, count in enumerate(episode_counts, start=1):
        rows.append(
            [
                str(index),
                Paragraph(count.episode_id, small_style),
                str(count.steps),
                Paragraph(count.file_path, small_style),
            ]
        )
    table = Table(
        rows,
        colWidths=[9 * mm, 40 * mm, 18 * mm, 113 * mm],
        repeatRows=1,
    )
    table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#2E5B88")),
                ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
                ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
                ("GRID", (0, 0), (-1, -1), 0.3, colors.HexColor("#B4C0D0")),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("ALIGN", (0, 1), (0, -1), "RIGHT"),
                ("ALIGN", (2, 1), (2, -1), "RIGHT"),
                ("LEFTPADDING", (0, 0), (-1, -1), 4),
                ("RIGHTPADDING", (0, 0), (-1, -1), 4),
                ("TOPPADDING", (0, 0), (-1, -1), 4),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
            ]
        )
    )
    story.append(table)

    document = SimpleDocTemplate(
        str(report_path),
        pagesize=A4,
        leftMargin=15 * mm,
        rightMargin=15 * mm,
        topMargin=15 * mm,
        bottomMargin=15 * mm,
        title="RLDS Near-Zero Action Conversion Report",
    )
    document.build(story)
    return report_path


def _expected_data_dir(args: argparse.Namespace) -> Path:
    dataset_name = args.dataset_name or DEFAULT_DATASET_NAME
    return args.output_dir.expanduser().resolve() / dataset_name / "1.0.0"


def main(argv: Sequence[str] | None = None) -> None:
    args = _parse_args(argv)
    _validate_args(args)
    plans: list[BagPlan] = discover_bags(args.input_dir, args.instruction_file_name)
    if not plans:
        raise SystemExit(f"No rosbag2 bags found under: {args.input_dir}")

    _print_plan(plans, fallback_instruction=args.instruction, show_bags=args.dry_run)
    if args.dry_run:
        return

    expected_data_dir = _expected_data_dir(args)
    if expected_data_dir.exists() and not args.overwrite:
        raise SystemExit(
            f"Dataset already exists: {expected_data_dir}\n"
            "Pass --overwrite to regenerate it with the selected near-zero-action settings."
        )

    config = _make_config(args, _instruction_overrides(plans))
    episode_transform = None
    if not args.keep_near_zero_actions:

        def episode_transform(episode: dict) -> dict:
            omit_near_zero_actions(
                episode,
                linear_threshold_m=args.linear_noop_threshold_m,
                angular_threshold_rad=args.angular_noop_threshold_rad,
            )
            return episode

    data_dir = convert_bags(
        [plan.path for plan in plans],
        output_dir=args.output_dir,
        config=config,
        overwrite=args.overwrite,
        dataset_name=args.dataset_name,
        episode_transform=episode_transform,
    )
    episode_counts = _read_episode_step_counts(data_dir)
    report_path = _write_pdf_report(
        data_dir=data_dir,
        episode_counts=episode_counts,
        filtering_enabled=not args.keep_near_zero_actions,
        linear_threshold_m=args.linear_noop_threshold_m,
        angular_threshold_rad=args.angular_noop_threshold_rad,
    )

    print(f"\nWrote RLDS/TFDS dataset to: {data_dir}")
    print(f"Wrote RLDS step-count report to: {report_path}")
    print(
        "Episodes: "
        f"{len(episode_counts)}; total RLDS steps: {sum(c.steps for c in episode_counts)}"
    )
    print("Load it with:")
    print(f"  tfds.builder_from_directory({str(data_dir)!r}).as_dataset(split='train')")


if __name__ == "__main__":
    main()
