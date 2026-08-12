## Commit and Push to GitLab
```bash
cd ~/franka_ros2_ws

# see what changed
git status    

git add .        
git commit -m "<message>"  
git push                       
```
## Build a Package
```bash
cd ~/franka_ros2_ws

colcon build --packages-select vla_data_collection --symlink-install


source install/setup.bash
```
## Check camera controls and formats
```bash
v4l2-ctl --list-ctrls --device /dev/video0

v4l2-ctl --list-formats-ext --device /dev/video0
```
Configure camera parameters live
```bash
ros2 run rqt_reconfigure rqt_reconfigure
```
## Data collection sequence (Streamlined & Automated)

To minimize terminal clutter, the data collection backend is combined into a single launch command, and the entire recording, trimming, zipping, conversion, and reporting pipeline is driven interactively.

### 1. Launch the Data Collection Backend (Terminal 1)
This launches the camera stream, MoveIt Servo controller, and gamepad teleop nodes in one go:
```bash
ros2 launch vla_data_collection vla_backend.launch.py robot_ip:=172.16.0.2
```
*Note: Pass `use_fake_hardware:=true` if running in simulation.*

### 2. Run the Episode Collection Pipeline (Terminal 2)
Run the interactive automation script. It prompts you for the instruction, suggests auto-incremented episode names, records the bag to `~/Dataset/Episodes`, trims it to `~/Dataset/Trimmed_episodes`, converts it to RLDS in `~/Dataset/Tensorflow`, and generates a report in `~/Dataset/Reports`:
```bash
ros2 run vla_data_collection collect_episode
```
You can also pass the language instruction directly as an argument:
```bash
ros2 run vla_data_collection collect_episode --instruction "pick up the red block"
```
### 3. Optional: If report and conversion to RLDS are not needed
To view the front camera feed:
```bash
--no-convert \
--no-report
```
### 4. Optional: Live Visualization (Terminal 3)
To view the front camera feed:
```bash
ros2 run rqt_image_view rqt_image_view
```
### 5. Batch conversion of rosbag2 files to a single RLDS
This performs a dry-run to get an overview of the rosbag2 files inside the folder:
```bash
ros2 run vla_data_collection rosbag2_folder_to_rlds \
  ~/Dataset/Trimmed_episodes \
  --output-dir ~/Dataset/Tensorflow \
  --dataset-name panda_vla_dataset \
  --camera-topic /vla_front_camera/image_raw \
  --dry-run
```
Converts the rosbag2 files inside the folder to a single RLDS
```bash
ros2 run vla_data_collection rosbag2_folder_to_rlds \
  ~/Dataset/Trimmed_episodes \
  --output-dir ~/Dataset/Tensorflow \
  --dataset-name panda_vla_dataset \
  --camera-topic /vla_front_camera/image_raw \
  --overwrite
```

Folder Structure -
```bash
~/Dataset/Trimmed_episodes/
  pick_up_red_block/
    episode_001/
    episode_002/
  place_blue_block/
    instruction.txt
    episode_003/
```
---

## Advanced: Manual Pipeline Steps

If you need to debug or run individual stages of the pipeline manually, follow the steps below.

### 1. Manual Recording
Ensure you have the instruction node running and the task instruction parameter set before recording:
```bash
ros2 run vla_data_collection instruction_node
ros2 param set /instruction_node instruction 'your task description'
```
Record the ROS 2 bag:
```bash
ros2 bag record \
  /vla_front_camera/image_raw/compressed \
  /joint_states \
  /franka_robot_state_broadcaster/robot_state \
  /servo_node/delta_twist_cmds \
  /vla/language_instruction \
  /tf \
  /tf_static \
  -o ~/Dataset/Episodes/ep_pick_red_block_0
```

### 2. Manual Trimming
Trim a raw recording down to the overlapping time window of VLA topics:
```bash 
ros2 run vla_data_collection rosbag2_trim_sync ~/Dataset/Episodes/ep_pick_red_block_0.zip --output ~/Dataset/Trimmed_episodes/tr_ep_pick_red_block_0 --overwrite
```

### 3. Manual Convert to RLDS
Build and source the workspace, install Python dataset dependencies:
```bash
cd ~/franka_ros2_ws
colcon build --packages-select vla_data_collection --symlink-install
source install/setup.bash
python3 -m pip install -r src/franka_ros2/vla_data_collection/requirements-rlds.txt
```
Convert the bag to RLDS:
```bash
ros2 run vla_data_collection rosbag2_to_rlds \
  ~/Dataset/Trimmed_episodes/tr_ep_pick_up_the_blue_cube_0.zip \
  --output-dir ~/Dataset/Tensorflow/tfds_pick_up_blue_block_0 \
  --image-size 224 \
  --instruction 'your task description' \
  --twist-action-mode integrated_delta \
  --overwrite
```
The stored 7-D action layout is `[dx, dy, dz, droll, dpitch, dyaw, gripper_delta]`.

Batch-convert a folder of many bags:
```bash
ros2 run vla_data_collection rosbag2_folder_to_rlds \
  ~/Dataset/Trimmed_episodes \
  --output-dir ~/Dataset/Tensorflow \
  --dataset-name panda_vla_dataset \
  --camera-topic /vla_front_camera/image_raw \
  --overwrite
```
If `~/Dataset/Trimmed_episodes` contains immediate subfolders, each subfolder is treated as one instruction group. For example, bags inside `pick_up_the_red_block/` get the instruction `pick up the red block`. To use a cleaner instruction than the folder name, put an `instruction.txt` file inside that subfolder.

Batch-convert while omitting stationary near-zero arm actions, but always retain binary gripper open/close transitions:
```bash
ros2 run vla_data_collection rosbag2_to_rlds_folder_noop \
  ~/Dataset/Trimmed_episodes \
  --output-dir ~/Dataset/Tensorflow \
  --dataset-name panda_vla_no_noops \
  --camera-topic /vla_front_camera/image_raw \
  --overwrite
```
The default no-op thresholds are translation norm below `0.0005` m and rotation norm below `0.005` rad. Override them with `--linear-noop-threshold-m` and `--angular-noop-threshold-rad`, or retain all synchronized samples with `--keep-near-zero-actions`. The command writes `rlds_noop_filter_report.pdf` inside the generated dataset directory with the final episode and step counts.

Validate the converted dataset:
```bash
python3 - <<'PY'
from pathlib import Path
import tensorflow_datasets as tfds

builder = tfds.builder_from_directory(
    str(Path("~/Dataset/Tensorflow/tfds_pick_red_block_0/franka_vla/1.0.0").expanduser())
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

### 4. Manual Timing and Alignment Report
Create an interactive HTML report to verify synchrony:
```bash
ros2 run vla_data_collection rlds_rosbag_report \
  ~/Dataset/Episodes/ep_pick_red_block_0.zip \
  --rlds ~/Dataset/Tensorflow/tfds_pick_red_block_0 \
  --output report_pick_red_block_0.html
```
Open the generated `report.html` in your web browser. It includes interactive plots comparing end-effector trajectories, joint states, gripper movements, sync delay analysis, and thumbnail steps.

---

## Training
DO NOT FORGET TO TWEAK THE SETTINGS ACCORDINGLY!!!

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

## Merge LoRA Weights

If you trained the model using OpenVLA-OFT, the training pipeline only saves the fine-tuned LoRA adapter weights (to save disk space) and does not save the base model's `config.json` or base weights. 
**Before deploying a fine-tuned model**, you must merge the LoRA weights back into the base model. If you skip this, the deployment script will fail because it cannot find the base model configuration.

Run the following command to merge the weights. The script will load the base model, apply your LoRA weights, and save the fully standalone model (including `config.json` and the weights) directly into your checkpoint directory:

```bash
cd ~/openvla-oft-main_modified_for_RLDS/openvla-oft-main
conda activate openvla-oft

python vla-scripts/merge_lora_weights_and_save.py \
    --base_checkpoint openvla/openvla-7b \
    --lora_finetuned_checkpoint_dir ~/openvla-oft-main_modified_for_RLDS/openvla-oft-main/runs/<YOUR_CHECKPOINT_DIR>
```

## Running VLA Inference

To run the full VLA stack in real-time inference (either in simulation or on the physical robot), you need to run three separate components in three distinct terminals.

### 1. Launch the Robot Backend (Terminal 1)
Start the robot connection (MoveIt Servo) and the camera driver:
```bash
ros2 launch vla_data_collection vla_backend.launch.py robot_ip:=172.16.0.2 use_fake_hardware:=false
```

### 2.a Start the VLA Deployment Server (FOR STOCK OPENVLA-OFT!!) (Terminal 2)
Activate the Python virtual environment containing the VLA dependencies and run the server script to host the model:
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
  --num_images_in_input 1 \
  --use_proprio True \
  --center_crop True \
  --lora_rank 32 \
  --unnorm_key libero_spatial_no_noops
```

### 2.b Start the VLA Deployment Server (FOR FINE-TUNED OPENVLA-OFT!!) (Terminal 2)
Activate the Python virtual environment containing the VLA dependencies and run the server script to host the model:
```bash
cd ~/openvla-oft-main_modified_for_RLDS/openvla-oft-main
conda activate openvla-oft

python vla-scripts/deploy.py \
  --host 0.0.0.0 \
  --port 8777 \
  --pretrained_checkpoint ~/openvla-oft-main_modified_for_RLDS/openvla-oft-main/runs/openvla-7b+franka_vla+b8+lr-0.0005+lora-r32+dropout-0.0--image_aug--franka_vla_v1--3945_chkpt \
  --use_l1_regression True \
  --use_diffusion False \
  --use_film False \
  --num_images_in_input 1 \
  --use_proprio True \
  --center_crop True \
  --lora_rank 32 \
  --unnorm_key franka_vla
```
### 3. Run the ROS 2 VLA Inference Bridge Node (Terminal 3)
Ensure the ROS workspace is sourced. You can run the node in two modes:

#### A. Testing / Dry Run (Safe)
This queries the model and prints the output commands without sending them to the robot hardware:
```bash
cd ~/franka_ros2_ws
source install/setup.bash

ros2 run vla_data_collection openvla_oft_inference_node --ros-args \
  -p dry_run:=true \
  -p server_url:=http://127.0.0.1:8777/act \
  -p duplicate_primary_as_wrist:=true \
  -p initial_instruction:="pick up the blue cube"
```

#### B. Live Control (Active)
This sends real motion and gripper commands directly to the hardware:
```bash
cd ~/franka_ros2_ws
source install/setup.bash

ros2 run vla_data_collection openvla_oft_inference_node --ros-args \
  -p dry_run:=false \
  -p server_url:=http://127.0.0.1:8777/act \
  -p duplicate_primary_as_wrist:=false \
  -p initial_instruction:="pick up the blue cube"
```

#### C. Live Control (Active), incrased speed
```bash
cd ~/franka_ros2_ws
source install/setup.bash

ros2 run vla_data_collection openvla_oft_inference_node --ros-args \
  -p dry_run:=false \
  -p server_url:=http://127.0.0.1:8777/act \
  -p duplicate_primary_as_wrist:=false \
  -p arm_action_mode:=integrated_delta \
  -p gripper_action_mode:=absolute_binary \
  -p max_linear_speed_mps:=0.05 \
  -p max_angular_speed_rps:=0.35 \
  -p command_duration_sec:=0.35 \
  -p query_period_sec:=0.35 \
  -p initial_instruction:="pick up the blue cube"
  
  
  
  
  
  
  ros2 run vla_data_collection openvla_oft_inference_node_more_action --ros-args \
  -p dry_run:=true \
  -p server_url:=http://127.0.0.1:8777/act \
  -p num_open_loop_steps:=4 \
  -p duplicate_primary_as_wrist:=false \
  -p arm_action_mode:=integrated_delta \
  -p gripper_action_mode:=absolute_binary \
  -p initial_instruction:="pick up the cube"
```
