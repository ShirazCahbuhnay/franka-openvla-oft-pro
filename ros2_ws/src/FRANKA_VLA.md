# Data Collection and VLA Operation

Use this guide to collect Franka demonstrations, convert them to RLDS, inspect
the resulting dataset, and run VLA inference.

## Contents
- [Data Collection](#data-collection)
- [Train OpenVLA-OFT](#train-openvla-oft)
- [Prepare a checkpoint for deployment](#prepare-a-checkpoint-for-deployment)
- [Run VLA inference](#run-vla-inference)

## Data Collection

### Launch the Backend
Always launch the backend before starting the data collection. This launches the Camera, MoveIt Servo and gamepad teleoperation.

```bash
ros2 launch vla_data_collection vla_backend.launch.py robot_ip:=172.16.0.2
```
For simulation, add `use_fake_hardware:=true`.

### Collect and Trim an Episode
```bash
ros2 run vla_data_collection collect_episode --no-convert \ --no-report
```
This command collects and trims an episode. If you want to test your setup you can ommit `--no-convert \ --no-report`.

The script will then automatically convert your episode to the RLDS format and generate a report comparing the collected topics and the RLDS steps. If you are planning on collecting an entire dataset, do not convert the trimmed episode to the RLDS format yet. The conversion must be done only once when all training episodes are collected. 

### Convert a Dataset Folder to RLDS 
It is crucial to organize your trimmed episodes folder before converting it. The conversion script can either extract the instruction from a .txt file inside the folder or if the .txt file is not present, it will use the folder name as the instruction. For example, bags in `pick_up_the_red_block/` receive the instruction `pick up the red block`.
Example:
```bash
Trimmed_episodes
|- batch1
|   |- instruction.txt
|   |- episode1
|   |- episode2
|   |- ...
|- pick_up_the_red_block
|   |- episode1
|   |- episode2
|   |- ...
```
Convert the bags to RLDS format:
```bash
ros2 run vla_data_collection rosbag2_to_rlds_folder_noop \
  ~/Dataset/Trimmed_episodes \
  --output-dir ~/Dataset/Tensorflow \
  --dataset-name panda_vla_dataset \
  --camera-topic /vla_front_camera/image_raw \
  --overwrite
```
The RLDS conversion script defaults to ommitting near-zero (no-op) arm actions. If you want to keep them, pass `--keep-near-zero-actions \` as an argument. 

Default no-op thresholds:

| Motion | Threshold |
| --- | ---: |
| Translation norm | Below `0.0005 m` |
| Rotation norm | Below `0.005 rad` |

Override these values with `--linear-noop-threshold-m` and
`--angular-noop-threshold-rad`.

The converter writes `rlds_noop_filter_report.pdf` inside the generated dataset
directory. The report includes the final episode and step counts. The step count is useful when calculating the finetuning parameters, such as the maximum steps for training. 

## Fine-Tuning OpenVLA-OFT
> **Before training:** Review the dataset name, batch size, training duration,
> checkpoint frequency, and image/proprioception settings. These values must
> match the dataset and available hardware.

The command below trains `openvla/openvla-7b` on the `franka_vla` dataset and
stores the run under `./runs`.

### Start Training

```bash
export WANDB_MODE=offline

  torchrun --standalone --nnodes 1 --nproc-per-node 3 vla-scripts/finetune.py \
    --vla_path openvla/openvla-7b \
    --data_root_dir ./Dataset/Tensorflow \
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
    --num_steps_before_decay 20000 \
    --max_steps 28800 \
    --save_freq 2880 \
    --save_latest_checkpoint_only False \
    --merge_lora_during_training False \
    --shuffle_buffer_size 13150 \
    --image_aug True \
    --lora_rank 32 \
    --lora_dropout 0 \
    --run_id_note franka_vla_v1
```

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
  --pretrained_checkpoint ~/openvla-oft-main_modified_for_RLDS/openvla-oft-main/runs/<YOUR_CHECKPOINT_DIR> \
  --use_l1_regression True \
  --use_diffusion False \
  --use_film False \
  --num_images_in_input 1 \
  --use_proprio True \
  --center_crop True \
  --lora_rank 32 \
  --unnorm_key franka_vla
```

### Terminal 3 — Start the ROS 2 Bridge

#### Safe Dry Run

Query the model and print its commands without sending them to the robot:

```bash
cd ~/franka_ros2_ws
source install/setup.bash

ros2 run vla_data_collection openvla_oft_inference_node_more_action --ros-args \
  -p server_url:=http://127.0.0.1:8777/act \
  -p dry_run:=true \
  -p query_period_sec:=0.35 \
  -p command_duration_sec:=0.20 \
  -p num_open_loop_steps:=4 \
  -p max_linear_speed_mps:=0.05 \
  -p max_angular_speed_rps:=0.35 \
  -p duplicate_primary_as_wrist:=false \
  -p initial_instruction:="pick up the blue cube" 
```

#### Live Control

After validating the dry-run output, enable robot motion:

```bash
cd ~/franka_ros2_ws
source install/setup.bash

ros2 run vla_data_collection openvla_oft_inference_node_more_action --ros-args \
  -p server_url:=http://127.0.0.1:8777/act \
  -p dry_run:=false \
  -p query_period_sec:=0.35 \
  -p command_duration_sec:=0.20 \
  -p num_open_loop_steps:=4 \
  -p max_linear_speed_mps:=0.05 \
  -p max_angular_speed_rps:=0.35 \
  -p duplicate_primary_as_wrist:=false \
  -p initial_instruction:="pick up the blue cube" 
```

These are the default values for all possible arguments:

```text
server_url                       http://127.0.0.1:8777/act
request_timeout_sec              10.0
dry_run                          true
camera_topic                     /vla_front_camera/image_raw
camera_compressed                false
joint_states_topic               /joint_states
robot_state_topic                /franka_robot_state_broadcaster/robot_state
instruction_topic                /vla/language_instruction
initial_instruction              ""
servo_topic                      /servo_node/delta_twist_cmds
gripper_action_name              /panda_gripper/move
command_frame                    panda_link0
query_period_sec                 1.0
servo_publish_period_sec         0.034
command_duration_sec             0.20
num_open_loop_steps              4
arm_action_mode                  integrated_delta
gripper_action_mode              absolute_binary
max_gripper_width_m              0.08
gripper_speed_mps                0.04
gripper_open_threshold           0.5
gripper_command_epsilon_m        0.004
gripper_command_cooldown_sec     0.50
max_linear_speed_mps             0.05
max_angular_speed_rps            0.20
image_size                       224
center_crop_image                true
duplicate_primary_as_wrist       false
max_image_age_sec                2.0
max_robot_state_age_sec          2.0
log_every_action                 true
log_full_action_chunk            true
```
The only parameter that must become non-empty before inference starts is the instruction, either through initial_instruction or a message on /vla/language_instruction.