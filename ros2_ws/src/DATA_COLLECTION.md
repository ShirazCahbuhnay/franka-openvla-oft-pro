# Data Collection and VLA Operation

This guide goes more in depth into the scripts used to collect Franka demonstrations, convert them to RLDS, inspect the resulting dataset, and run VLA inference. For a quick start, refer to [FRANKA_VLA.md](FRANKA_VLA.md)

The standard collection workflow uses:

- the raw `/vla_front_camera/image_raw` camera stream
- `rgb8` images at `1024 × 768` and approximately `10 Hz`
- `~/Dataset` for recordings, trimmed bags, RLDS datasets, and reports
- a Franka robot at `172.16.0.2`

> **Safety:** Start inference with `dry_run:=true`. Switch to live control only
> after checking the predicted actions, confirming the active publishers, and
> clearing the robot workspace.

## Contents

- [Quick start: automated collection](#quick-start-automated-collection)
- [Manual collection and conversion](#manual-collection-and-conversion)
- [Train OpenVLA-OFT](#train-openvla-oft)
- [Prepare a checkpoint for deployment](#prepare-a-checkpoint-for-deployment)
- [Run VLA inference](#run-vla-inference)
- [Useful maintenance commands](#useful-maintenance-commands)

## Quick Start: Automated Collection

The automated workflow needs two terminals. A third terminal is optional for
the live camera view.

| Terminal | Purpose | Command |
| --- | --- | --- |
| 1 | Camera, MoveIt Servo, and gamepad teleoperation | `ros2 launch ... vla_backend.launch.py` |
| 2 | Record, trim, archive, convert, and report | `ros2 run ... collect_episode` |
| 3 | Optional live camera view | `ros2 run rqt_image_view rqt_image_view` |

### Terminal 1 — Launch the Backend

Start the camera stream, MoveIt Servo controller, and gamepad teleoperation
nodes:

```bash
ros2 launch vla_data_collection vla_backend.launch.py robot_ip:=172.16.0.2
```

For simulation, add `use_fake_hardware:=true`.

### Terminal 2 — Collect an Episode

Start the interactive collection pipeline:

```bash
ros2 run vla_data_collection collect_episode
```

The script:

1. prompts for a language instruction
2. publishes the instruction at `5 Hz`
3. suggests an auto-incremented episode name
4. records the ROS 2 bag
5. trims and archives the recording
6. converts the trimmed bag to RLDS
7. generates a trimmed-bag-versus-RLDS report

| Output | Location |
| --- | --- |
| Raw recordings | `~/Dataset/Episodes` |
| Trimmed recordings | `~/Dataset/Trimmed_episodes` |
| RLDS datasets | `~/Dataset/Tensorflow` |
| Validation reports | `~/Dataset/Reports` |

To provide the instruction directly:

```bash
ros2 run vla_data_collection collect_episode \
  --instruction "pick up the red block"
```

### Terminal 3 — View the Camera (Optional)

```bash
ros2 run rqt_image_view rqt_image_view
```

## Manual Collection and Conversion

Use the manual workflow to debug a stage or rerun only part of the pipeline.

### 1. Record a ROS 2 Bag

Start the instruction node and set the task description:

```bash
ros2 run vla_data_collection instruction_node
ros2 param set /instruction_node instruction 'your task description'
```

Record the required topics:

```bash
ros2 bag record \
  /vla_front_camera/image_raw \
  /joint_states \
  /franka_robot_state_broadcaster/robot_state \
  /servo_node/delta_twist_cmds \
  /vla/language_instruction \
  /tf \
  /tf_static \
  -o ~/Dataset/Episodes/ep_pick_red_block_0
```

### 2. Trim the Recording

Trim the raw recording to the overlapping time window of the VLA topics:

```bash
ros2 run vla_data_collection rosbag2_trim_sync \
  ~/Dataset/Episodes/ep_pick_red_block_0.zip \
  --output ~/Dataset/Trimmed_episodes/tr_ep_pick_red_block_0 \
  --overwrite
```

If no instruction message remains inside the trimmed window, the tool preserves
the latest instruction published before the trim start.

### 3. Convert One Bag to RLDS

Build and source the package, then install the dataset dependencies:

```bash
cd ~/franka_ros2_ws
colcon build --packages-select vla_data_collection --symlink-install
source install/setup.bash
python3 -m pip install \
  -r src/franka_ros2/vla_data_collection/requirements-rlds.txt
```

Convert the trimmed bag:

```bash
ros2 run vla_data_collection rosbag2_to_rlds \
  ~/Dataset/Trimmed_episodes/tr_ep_pick_red_block_0.zip \
  --output-dir ~/Dataset/Tensorflow/tfds_pick_red_block_0 \
  --image-size 224 \
  --instruction 'your task description' \
  --twist-action-mode integrated_delta \
  --overwrite
```

The stored action is a seven-dimensional vector:

```text
[dx, dy, dz, droll, dpitch, dyaw, gripper_delta]
```

#### Camera Format

The default camera topic is `/vla_front_camera/image_raw`. The converter
supports these common raw encodings:

- `rgb8`
- `bgr8`
- `yuyv` / `yuy2`
- `uyvy`

For an older recording that uses compressed images, pass
`--camera-topic /vla_front_camera/image_raw/compressed` to the trim, conversion,
and report commands.

### 4. Convert a Folder to RLDS

#### Keep Every Synchronized Step

```bash
ros2 run vla_data_collection rosbag2_to_rlds_folder_noop \
  ~/Dataset/Trimmed_episodes \
  --output-dir ~/Dataset/Tensorflow \
  --dataset-name panda_vla_dataset \
  --camera-topic /vla_front_camera/image_raw \
  --keep-near-zero-actions \
  --overwrite
```

Immediate subfolders inside `~/Dataset/Trimmed_episodes` are treated as
instruction groups. For example, bags in `pick_up_the_red_block/` receive the
instruction `pick up the red block`.

To override an instruction inferred from a folder name, add an
`instruction.txt` file inside that folder.

#### Remove Stationary Arm Steps

The same converter can omit near-zero arm actions while always retaining binary
gripper open/close transitions:

```bash
ros2 run vla_data_collection rosbag2_to_rlds_folder_noop \
  ~/Dataset/Trimmed_episodes \
  --output-dir ~/Dataset/Tensorflow \
  --dataset-name panda_vla_no_noops \
  --camera-topic /vla_front_camera/image_raw \
  --overwrite
```

Default no-op thresholds:

| Motion | Threshold |
| --- | ---: |
| Translation norm | Below `0.0005 m` |
| Rotation norm | Below `0.005 rad` |

Override these values with `--linear-noop-threshold-m` and
`--angular-noop-threshold-rad`. To keep every synchronized sample, add
`--keep-near-zero-actions`.

The converter writes `rlds_noop_filter_report.pdf` inside the generated dataset
directory. The report includes the final episode and step counts.

### 5. Validate the Dataset

Inspect the first step of the first episode:

```bash
python3 - <<'PY'
from pathlib import Path
import tensorflow_datasets as tfds

builder = tfds.builder_from_directory(
    str(
        Path(
            "~/Dataset/Tensorflow/"
            "tfds_pick_red_block_0/franka_vla/1.0.0"
        ).expanduser()
    )
)
ds = builder.as_dataset(split="train")
episode = next(iter(ds))
step = next(iter(episode["steps"]))

print(step["observation"]["image"].shape)
print(step["observation"]["state"].shape)
print(step["action"].shape)
print(step["language_instruction"].numpy())
PY
```

### 6. Generate a Timing and Alignment Report

Create the interactive HTML report:

```bash
ros2 run vla_data_collection rlds_rosbag_report \
  ~/Dataset/Trimmed_episodes/tr_ep_pick_red_block_0.zip \
  --rlds ~/Dataset/Tensorflow/tfds_pick_red_block_0 \
  --output report_pick_red_block_0.html
```

Open `report_pick_red_block_0.html` in a browser. It compares:

- end-effector trajectories
- joint states
- gripper motion
- synchronization delays
- thumbnail steps

## Train OpenVLA-OFT

> **Before training:** Review the dataset name, batch size, training duration,
> checkpoint frequency, and image/proprioception settings. These values must
> match the dataset and available hardware.

The command below trains `openvla/openvla-7b` on the `franka_vla` dataset and
stores the run under `./runs`.

### Start Training

```bash
export WANDB_MODE=offline

torchrun --standalone --nnodes 1 --nproc-per-node 1 vla-scripts/finetune.py \
  --vla_path openvla/openvla-7b \
  --data_root_dir ~/Dataset/Tensorflow \
  --dataset_name franka_vla \
  --run_root_dir ./runs \
  --use_l1_regression True \
  --use_diffusion False \
  --use_film False \
  --num_images_in_input 1 \
  --use_proprio True \
  --batch_size 8 \
  --grad_accumulation_steps 1 \
  --learning_rate 5e-4 \
  --num_steps_before_decay 300 \
  --max_steps 450 \
  --save_freq 150 \
  --save_latest_checkpoint_only False \
  --merge_lora_during_training True \
  --shuffle_buffer_size 200 \
  --image_aug True \
  --lora_rank 32 \
  --lora_dropout 0.0 \
  --run_id_note franka_vla
```

### Settings at a Glance

| Setting | Value |
| --- | ---: |
| Dataset | `franka_vla` |
| Batch size | `8` |
| Gradient accumulation | `1` step |
| Learning rate | `5e-4` |
| Learning-rate decay starts | Step `300` |
| Training ends | Step `450` |
| Checkpoint interval | Every `150` steps |
| LoRA rank | `32` |
| W&B mode | Offline |

## Prepare a Checkpoint for Deployment

### Merge the LoRA Weights

OpenVLA-OFT saves the fine-tuned LoRA adapter weights without duplicating the
base model's `config.json` and weights. Before deploying a fine-tuned model,
merge the LoRA adapter back into the base model.

> **Required for deployment:** Without the merge step, the deployment script
> cannot find the base model configuration.

The merge script:

1. loads the `openvla/openvla-7b` base model
2. applies the fine-tuned LoRA weights
3. writes a standalone model, including `config.json` and the model weights,
   into the checkpoint directory

Replace `<YOUR_CHECKPOINT_DIR>` with the directory created by the training run:

```bash
cd ~/openvla-oft-main_modified_for_RLDS/openvla-oft-main
conda activate openvla-oft

python vla-scripts/merge_lora_weights_and_save.py \
  --base_checkpoint openvla/openvla-7b \
  --lora_finetuned_checkpoint_dir \
  ~/openvla-oft-main_modified_for_RLDS/openvla-oft-main/runs/<YOUR_CHECKPOINT_DIR>
```

## Run VLA Inference

Real-time inference uses three terminals:

```text
Deployment server  →  ROS 2 inference bridge  →  MoveIt Servo and gripper
```

### Terminal 1 — Launch the Robot Backend

Start the robot connection, MoveIt Servo, and camera driver:

```bash
ros2 launch vla_data_collection vla_backend.launch.py \
  robot_ip:=172.16.0.2 \
  use_fake_hardware:=false
```

### Terminal 2 — Start the Deployment Server

Activate the OpenVLA-OFT environment and host the model:

```bash
cd ~/openvla-oft-main_modified_for_RLDS/openvla-oft-main
conda activate openvla-oft

python vla-scripts/deploy.py \
  --host 0.0.0.0 \
  --port 8777 \
  --pretrained_checkpoint moojink/openvla-7b-oft-finetuned-libero-spatial \
  --use_l1_regression True \
  --use_diffusion False \
  --use_film False \
  --num_images_in_input 2 \
  --use_proprio True \
  --center_crop True \
  --lora_rank 32 \
  --unnorm_key libero_spatial_no_noops
```

### Terminal 3 — Start the ROS 2 Bridge

#### Safe Dry Run

Query the model and print its commands without sending them to the robot:

```bash
cd ~/franka_ros2_ws
source install/setup.bash

ros2 run vla_data_collection openvla_oft_inference_node_more_action --ros-args \
  -p dry_run:=true \
  -p server_url:=http://127.0.0.1:8777/act \
  -p num_open_loop_steps:=4 \
  -p duplicate_primary_as_wrist:=true \
  -p initial_instruction:="pick up the blue cube"
```

#### Live Control

After validating the dry-run output, enable robot motion:

```bash
cd ~/franka_ros2_ws
source install/setup.bash

ros2 run vla_data_collection openvla_oft_inference_node_more_action --ros-args \
  -p dry_run:=false \
  -p server_url:=http://127.0.0.1:8777/act \
  -p num_open_loop_steps:=4 \
  -p duplicate_primary_as_wrist:=true \
  -p initial_instruction:="pick up the blue cube"
```

## Useful Maintenance Commands

### Build the Package

```bash
cd ~/franka_ros2_ws
colcon build --packages-select vla_data_collection --symlink-install
source install/setup.bash
```

### Inspect Camera Controls and Formats

```bash
v4l2-ctl --list-ctrls --device /dev/video0
v4l2-ctl --list-formats-ext --device /dev/video0
```

To adjust camera parameters while the driver is running:

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

### Commit and Push Changes

```bash
cd ~/franka_ros2_ws/src/franka_ros2
git status
git add .
git commit -m "<message>"
git push
```
