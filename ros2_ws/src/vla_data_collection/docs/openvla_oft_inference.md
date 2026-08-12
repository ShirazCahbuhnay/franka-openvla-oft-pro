# OpenVLA-OFT Inference In This Workspace

This workspace uses a two-process deployment shape:

1. OpenVLA-OFT server loads the 7B policy and exposes `POST /act`.
2. ROS 2 bridge node reads camera, robot state, joint state, and instruction topics, calls the server, then converts the 7-D action into:
   - `geometry_msgs/TwistStamped` on `/servo_node/delta_twist_cmds`
   - `franka_msgs/action/Move` goals on `/panda_gripper/move`

The bridge node is:

```bash
ros2 run vla_data_collection openvla_oft_inference_node
```

It defaults to `dry_run:=true`, so the first run does not command the robot.

## What The Node Sends To OpenVLA-OFT

The bridge sends the same observation fields used by `vla-scripts/deploy.py`:

```python
{
    "full_image": uint8_rgb_image,
    "state": [eef_x, eef_y, eef_z, eef_roll, eef_pitch, eef_yaw, 0.0, gripper_width],
    "instruction": "your language instruction",
}
```

If `duplicate_primary_as_wrist:=true`, it also sends:

```python
{
    "wrist_image": same_uint8_rgb_image
}
```

That option is useful only for testing public stock OpenVLA-OFT checkpoints that expect one third-person image plus one wrist image. It is not the right setting for your custom `franka_vla` training run, which used `--num_images_in_input 1`.

## Build The ROS Workspace

From the ROS workspace root:

```bash
cd /Users/gauthamvishnuchandran/Downloads/workspace_inference/workspace
colcon build --symlink-install --packages-select franka_msgs franka_moveit_config vla_data_collection
source install/setup.bash
```

The bridge node needs a few Python client packages in the ROS Python environment:

```bash
python3 -m pip install requests json-numpy opencv-python
```

## Start The Robot/Servo Side

For fake hardware or first plumbing tests:

```bash
ros2 launch vla_data_collection vla_backend.launch.py \
  use_fake_hardware:=true \
  launch_controller_teleop:=false \
  use_ps5_6dof_teleop:=false
```

For the real Panda, keep teleop disabled while the policy is publishing:

```bash
ros2 launch vla_data_collection vla_backend.launch.py \
  robot_ip:=172.16.0.2 \
  use_fake_hardware:=false \
  launch_controller_teleop:=false \
  use_ps5_6dof_teleop:=false
```

Set or update the instruction:

```bash
ros2 run vla_data_collection instruction_node --ros-args \
  -p instruction:="pick up the banana"
```

Or, if `instruction_node` is already running:

```bash
ros2 param set /instruction_node instruction "pick up the banana"
```

## Option A: Stock OpenVLA-OFT Dry Run

This is only for checking that the server, camera, state, JSON transport, and ROS bridge work. The public LIBERO checkpoint is not trained on your Franka camera/task distribution, so do not expect useful robot behavior from it.

In the OpenVLA-OFT checkout:

```bash
cd /Users/gauthamvishnuchandran/Downloads/workspace_inference/modified_openvla_oft/openvla-oft-main
conda activate openvla-oft
python -m pip install uvicorn fastapi json-numpy

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

In another terminal with the ROS workspace sourced:

```bash
cd /Users/gauthamvishnuchandran/Downloads/workspace_inference/workspace
source install/setup.bash

ros2 run vla_data_collection openvla_oft_inference_node --ros-args \
  -p dry_run:=true \
  -p server_url:=http://127.0.0.1:8777/act \
  -p duplicate_primary_as_wrist:=true \
  -p arm_action_mode:=integrated_delta \
  -p gripper_action_mode:=absolute_binary \
  -p initial_instruction:="pick up the banana"
```

You should see log lines with `action=[...]`, `twist=[...]`, and `dry_run=True`. The node will not publish motion commands in this mode.

## Option B: Your Trained Franka Checkpoint

Use this after fine-tuning on `franka_vla`.

If you trained with `--merge_lora_during_training False`, merge the LoRA adapter into the base model before deployment:

```bash
cd /Users/gauthamvishnuchandran/Downloads/workspace_inference/modified_openvla_oft/openvla-oft-main
conda activate openvla-oft

python vla-scripts/merge_lora_weights_and_save.py \
  --base_checkpoint openvla/openvla-7b \
  --lora_finetuned_checkpoint_dir ./runs/YOUR_FRANKA_CHECKPOINT_DIR
```

Then start the server with the Franka checkpoint:

```bash
python vla-scripts/deploy.py \
  --host 0.0.0.0 \
  --port 8777 \
  --pretrained_checkpoint ./runs/YOUR_FRANKA_CHECKPOINT_DIR \
  --use_l1_regression True \
  --use_diffusion False \
  --use_film False \
  --num_images_in_input 1 \
  --use_proprio True \
  --center_crop True \
  --lora_rank 32 \
  --unnorm_key franka_vla
```

Run the ROS bridge in dry-run first:

```bash
ros2 run vla_data_collection openvla_oft_inference_node --ros-args \
  -p dry_run:=true \
  -p server_url:=http://127.0.0.1:8777/act \
  -p duplicate_primary_as_wrist:=false \
  -p arm_action_mode:=integrated_delta \
  -p gripper_action_mode:=absolute_binary \
  -p initial_instruction:="pick up the banana"
```

If your RLDS conversion used `--gripper-action-mode absolute_width`, run the node with:

```bash
-p gripper_action_mode:=absolute_width
```

Only after the dry-run actions look reasonable, command the robot with conservative limits:

```bash
ros2 run vla_data_collection openvla_oft_inference_node --ros-args \
  -p dry_run:=false \
  -p server_url:=http://127.0.0.1:8777/act \
  -p duplicate_primary_as_wrist:=false \
  -p arm_action_mode:=integrated_delta \
  -p gripper_action_mode:=absolute_binary \
  -p max_linear_speed_mps:=0.02 \
  -p max_angular_speed_rps:=0.10 \
  -p command_duration_sec:=0.20 \
  -p query_period_sec:=1.0 \
  -p initial_instruction:="pick up the banana"
```

## Important Parameters

```text
dry_run
  true logs actions only; false commands Servo and gripper.

server_url
  OpenVLA-OFT server endpoint, normally http://127.0.0.1:8777/act.

arm_action_mode
  integrated_delta: action[:6] is treated like the converter default twist * camera_dt.
  raw_twist: action[:6] is already a TwistStamped velocity command.

gripper_action_mode
  absolute_binary: action[6] >= threshold means open, otherwise close.
  absolute_width: action[6] is normalized width in [0, 1].
  delta: action[6] is physical width delta in meters.

duplicate_primary_as_wrist
  true sends the main camera as a fake wrist image for stock LIBERO checkpoints.
  false for your one-camera Franka checkpoint.

max_linear_speed_mps / max_angular_speed_rps
  safety clamps applied after decoding the policy action.
```

## Quick Checks

Verify the required ROS topics:

```bash
ros2 topic hz /vla_front_camera/image_raw
ros2 topic hz /franka_robot_state_broadcaster/robot_state
ros2 topic echo /joint_states --once
ros2 topic echo /vla/language_instruction --once
ros2 action list | grep panda_gripper
```

When `dry_run:=false`, verify policy commands are reaching Servo:

```bash
ros2 topic echo /servo_node/delta_twist_cmds --once
```

## Troubleshooting

If the server says `Action un-norm key ... not found`, use the dataset key that matches the checkpoint:

```text
stock LIBERO spatial: --unnorm_key libero_spatial_no_noops
your Franka dataset:  --unnorm_key franka_vla
```

If the server cannot find `action_head` or `proprio_projector`, you are probably pointing it at an unmerged adapter-only checkpoint or the wrong checkpoint directory.

If the policy fights teleop, relaunch Servo with `launch_controller_teleop:=false` and `use_ps5_6dof_teleop:=false`.

If gripper commands are wrong, re-check the RLDS conversion mode. The node parameter `gripper_action_mode` must match the conversion mode used to create the training dataset.
