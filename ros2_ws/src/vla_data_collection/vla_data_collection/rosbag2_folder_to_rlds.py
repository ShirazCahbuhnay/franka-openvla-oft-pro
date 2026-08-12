"""Batch-convert a folder of rosbag2 episodes into one RLDS/TFDS dataset."""

from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

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
    convert_bags,
)


DEFAULT_IMAGE_SIZE = 256


@dataclass(frozen=True)
class BagPlan:
    path: Path
    instruction: str | None
    group_name: str | None


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


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Discover rosbag2 episodes under a folder and convert them into one "
            "RLDS/TFDS dataset. Immediate subfolders are treated as instruction groups."
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
        help="TFDS dataset name, for example panda_vla_dataset. Defaults to franka_vla.",
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
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print discovered bags and instructions without converting.",
    )
    return parser.parse_args(argv)


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


def main(argv: Sequence[str] | None = None) -> None:
    args = _parse_args(argv)
    plans = discover_bags(args.input_dir, args.instruction_file_name)
    if not plans:
        raise SystemExit(f"No rosbag2 bags found under: {args.input_dir}")

    _print_plan(plans, fallback_instruction=args.instruction, show_bags=args.dry_run)
    if args.dry_run:
        return

    config = _make_config(args, _instruction_overrides(plans))
    data_dir = convert_bags(
        [plan.path for plan in plans],
        output_dir=args.output_dir,
        config=config,
        overwrite=args.overwrite,
        dataset_name=args.dataset_name,
    )

    print(f"\nWrote RLDS/TFDS dataset to: {data_dir}")
    print("Load it with:")
    print(f"  tfds.builder_from_directory({str(data_dir)!r}).as_dataset(split='train')")


if __name__ == "__main__":
    main()
