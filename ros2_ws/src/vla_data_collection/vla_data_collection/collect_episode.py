#!/usr/bin/env python3
"""Interactive script to automate and streamline the ROS 2 bag recording and RLDS conversion pipeline.

This script runs an interactive session to record an episode, automatically zips
the raw rosbag, trims it, converts it to RLDS format, and generates a timing/alignment report.
"""

import argparse
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import zipfile
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String


class InstructionPublisher(Node):
    """ROS 2 Node that publishes the language instruction on /vla/language_instruction."""

    def __init__(self, instruction: str):
        super().__init__('collect_episode_instruction_publisher')
        # Setup transient local durability to match instruction_node.py
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.publisher = self.create_publisher(String, '/vla/language_instruction', qos_profile)
        self.msg = String()
        self.msg.data = instruction
        
        self.get_logger().info(f"Publishing instruction: '{instruction}'")
        self.publish()
        
        # Publish at 5 Hz so the instruction survives ordinary trim windows.
        self.timer = self.create_timer(0.2, self.publish)

    def publish(self):
        self.publisher.publish(self.msg)


def sanitize_instruction(instruction: str) -> str:
    """Convert language instruction into a clean snake_case string for filenames."""
    s = instruction.lower()
    # Remove non-alphanumeric/spaces
    s = re.sub(r'[^a-z0-9\s_]', '', s)
    # Replace spaces with single underscore
    s = re.sub(r'\s+', '_', s)
    return s.strip('_')


def get_next_episode_index(base_name: str, episodes_dir: Path) -> int:
    """Find the next incremental index for base_name in episodes_dir to prevent overwrites."""
    index = 0
    while True:
        candidate_dir = episodes_dir / f"ep_{base_name}_{index}"
        candidate_zip = episodes_dir / f"ep_{base_name}_{index}.zip"
        if not candidate_dir.exists() and not candidate_zip.exists():
            return index
        index += 1


def zip_directory(dir_path: Path, zip_path: Path):
    """Zip a directory including its root name folder, then clean up the directory."""
    print(f"Zipping {dir_path.name} to {zip_path.name}...")
    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
        for file_path in dir_path.rglob('*'):
            if file_path.is_file():
                # Store with the folder name as root inside the zip
                zipf.write(file_path, file_path.relative_to(dir_path.parent))
    print(f"Cleaning up unzipped directory {dir_path}...")
    shutil.rmtree(dir_path)


def run_command(cmd: list[str], description: str):
    """Run a command with subprocess and check for success."""
    print(f"\n>>> Running: {description}...")
    print(f"Command: {' '.join(cmd)}")
    result = subprocess.run(cmd)
    if result.returncode != 0:
        raise RuntimeError(f"Command failed with exit code {result.returncode}: {' '.join(cmd)}")


def main():
    parser = argparse.ArgumentParser(description="Automated ROS2 bag recording & RLDS conversion pipeline.")
    parser.add_argument("-i", "--instruction", type=str, default=None, help="Language instruction for the task.")
    parser.add_argument("-n", "--name", type=str, default=None, help="Base name of the episode (optional).")
    parser.add_argument("--episodes-dir", type=str, default="~/Dataset/Episodes", help="Directory for raw rosbags.")
    parser.add_argument("--trimmed-dir", type=str, default="~/Dataset/Trimmed_episodes", help="Directory for trimmed rosbags.")
    parser.add_argument("--rlds-dir", type=str, default="~/Dataset/Tensorflow", help="Directory for RLDS datasets.")
    parser.add_argument("--reports-dir", type=str, default="~/Dataset/Reports", help="Directory for timing/alignment reports.")
    parser.add_argument(
        "--camera-topic",
        type=str,
        default="/vla_front_camera/image_raw",
        help=(
            "Camera topic to record and convert. Defaults to the raw rgb8 "
            "sensor_msgs/Image stream used by the current dataset."
        ),
    )
    parser.add_argument("--image-size", type=int, default=224, help="Image size for RLDS converter (default: 224).")
    parser.add_argument("--no-trim", action="store_true", help="Skip trimming step.")
    parser.add_argument("--no-convert", action="store_true", help="Skip RLDS conversion step.")
    parser.add_argument("--no-report", action="store_true", help="Skip report generation step.")
    parser.add_argument("--no-zip", action="store_true", help="Do not zip bag directories.")
    args = parser.parse_args()

    # Resolve paths
    episodes_dir = Path(args.episodes_dir).expanduser().resolve()
    trimmed_dir = Path(args.trimmed_dir).expanduser().resolve()
    rlds_dir = Path(args.rlds_dir).expanduser().resolve()
    reports_dir = Path(args.reports_dir).expanduser().resolve()

    # Create directories
    episodes_dir.mkdir(parents=True, exist_ok=True)
    trimmed_dir.mkdir(parents=True, exist_ok=True)
    rlds_dir.mkdir(parents=True, exist_ok=True)
    reports_dir.mkdir(parents=True, exist_ok=True)

    # Get language instruction interactively if not provided
    instruction = args.instruction
    if not instruction:
        while not instruction:
            try:
                instruction = input("\nEnter language instruction: ").strip()
            except (KeyboardInterrupt, EOFError):
                print("\nExiting.")
                return

    # Determine episode base name
    base_name = args.name
    if not base_name:
        base_name = sanitize_instruction(instruction)
        if not base_name:
            base_name = "episode"

    # Determine episode index and names
    index = get_next_episode_index(base_name, episodes_dir)
    raw_name = f"ep_{base_name}_{index}"
    trimmed_name = f"tr_ep_{base_name}_{index}"
    rlds_subdir = f"tfds_{base_name}_{index}"
    report_name = f"report_{base_name}_{index}.html"

    raw_bag_dir = episodes_dir / raw_name
    raw_bag_zip = episodes_dir / f"{raw_name}.zip"
    trimmed_bag_dir = trimmed_dir / trimmed_name
    trimmed_bag_zip = trimmed_dir / f"{trimmed_name}.zip"
    rlds_output_dir = rlds_dir / rlds_subdir
    report_output_path = reports_dir / report_name

    # Initialize ROS2 Node in background to publish instruction
    rclpy.init()
    node = InstructionPublisher(instruction)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Prepare topics to record
    topics = [
        args.camera_topic,
        "/joint_states",
        "/franka_robot_state_broadcaster/robot_state",
        "/servo_node/delta_twist_cmds",
        "/vla/language_instruction",
        "/tf",
        "/tf_static",
    ]

    record_cmd = [
        "ros2", "bag", "record",
        *topics,
        "-o", str(raw_bag_dir)
    ]

    recording_proc = None
    try:
        print("\n" + "="*50)
        print(f"READY TO RECORD EPISODE")
        print(f"  Name:        {raw_name}")
        print(f"  Instruction: '{instruction}'")
        print(f"  Output:      {raw_bag_dir}")
        print("="*50)
        
        input("\nPress ENTER to START recording...")
        
        print("\nRecording started! Perform the action now.")
        print("Press ENTER (or Ctrl+C) to STOP recording...")
        
        # Start recording process in a new process group so we can SIGINT it cleanly
        recording_proc = subprocess.Popen(record_cmd, preexec_fn=os.setsid)
        
        start_time = time.time()
        # Wait for user input to stop
        try:
            input()
        except (KeyboardInterrupt, EOFError):
            print("\nStop command received.")
            
        duration = time.time() - start_time
        print(f"\nRecording stopped. Duration: {duration:.2f} seconds.")

    finally:
        # Gracefully stop the recording
        if recording_proc and recording_proc.poll() is None:
            print("Stopping ros2 bag record process...")
            try:
                os.killpg(os.getpgid(recording_proc.pid), signal.SIGINT)
                recording_proc.wait(timeout=10.0)
                print("ros2 bag record exited cleanly.")
            except Exception as e:
                print(f"Error stopping bag recorder: {e}")
                recording_proc.kill()
        
        # Clean up ROS2 node
        node.destroy_node()
        rclpy.try_shutdown()

    # Verify raw bag directory exists
    if not raw_bag_dir.exists():
        print(f"Error: Rosbag directory {raw_bag_dir} was not created. Recording failed.")
        return

    # Post-processing pipeline
    try:
        # Zip the raw recording if requested
        input_bag_path = raw_bag_dir
        if not args.no_zip:
            zip_directory(raw_bag_dir, raw_bag_zip)
            input_bag_path = raw_bag_zip

        # 1. Trim Episode
        if not args.no_trim:
            trim_cmd = [
                "ros2", "run", "vla_data_collection", "rosbag2_trim_sync",
                str(input_bag_path),
                "--output", str(trimmed_bag_dir),
                "--camera-topic", args.camera_topic,
                "--overwrite"
            ]
            run_command(trim_cmd, "Trimming ROS 2 bag to synchronized time window")
            
            trimmed_input_path = trimmed_bag_dir
            if not args.no_zip:
                zip_directory(trimmed_bag_dir, trimmed_bag_zip)
                trimmed_input_path = trimmed_bag_zip
        else:
            trimmed_input_path = input_bag_path

        # 2. Convert to RLDS
        if not args.no_convert:
            convert_cmd = [
                "ros2", "run", "vla_data_collection", "rosbag2_to_rlds",
                str(trimmed_input_path),
                "--output-dir", str(rlds_output_dir),
                "--camera-topic", args.camera_topic,
                "--image-size", str(args.image_size),
                "--instruction", instruction,
                "--overwrite"
            ]
            run_command(convert_cmd, "Converting trimmed episode to RLDS/TFDS format")

        # 3. Generate Timing/Alignment Report
        if not args.no_report and not args.no_convert:
            report_cmd = [
                "ros2", "run", "vla_data_collection", "rlds_rosbag_report",
                str(trimmed_input_path),
                "--rlds", str(rlds_output_dir),
                "--camera-topic", args.camera_topic,
                "--output", str(report_output_path)
            ]
            run_command(report_cmd, "Generating timing and alignment report")

        # Final Success Summary
        print("\n" + "="*50)
        print("EPISODE COLLECTION PIPELINE COMPLETE!")
        print(f"  Raw Recording:    {input_bag_path}")
        if not args.no_trim:
            print(f"  Trimmed Bag:      {trimmed_input_path}")
        if not args.no_convert:
            print(f"  RLDS Output:      {rlds_output_dir}")
        if not args.no_report and not args.no_convert:
            print(f"  Alignment Report: {report_output_path}")
        print("="*50 + "\n")

    except Exception as e:
        print(f"\nPipeline execution failed: {e}", file=sys.stderr)


if __name__ == "__main__":
    main()
