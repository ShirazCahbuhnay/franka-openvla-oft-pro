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