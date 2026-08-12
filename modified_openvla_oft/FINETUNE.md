# Franka Panda OpenVLA-OFT Fine-Tuning Notes

This document explains what modifications were made to the original openvla-oft scripts so it can load and fine-tune on the recorded Franka Panda dataset. Refer to the ROS2 workspace folder for instructions on finetuning and running inference. 

The original openvla-oft repository:

```text
https://github.com/moojink/openvla-oft
```

Only three source files contain meaningful code changes:

```text
openvla-oft-main/prismatic/vla/datasets/rlds/oxe/configs.py
openvla-oft-main/prismatic/vla/datasets/rlds/oxe/transforms.py
openvla-oft-main/prismatic/vla/datasets/rlds/oxe/mixtures.py
```
## Setup Environment
 
Refer to [SETUP.md](openvla-oft-main/SETUP.md) to setup the necessary OpenVLA environment for running the fine-tuning and inference scripts. 

## Fine-Tuning Command

This training command should be run after making the necessary modifications explained below. 

```bash
torchrun --standalone --nnodes 1 --nproc-per-node 1 vla-scripts/finetune.py \
      --vla_path openvla/openvla-7b \
      --data_root_dir ~/Dataset/OpenVLA_TFDS \
      --dataset_name franka_vla \
      --run_root_dir ./runs \
      --use_l1_regression True \
      --use_diffusion False \
      --use_film False \
      --num_images_in_input 1 \
      --use_proprio True \
      --batch_size 8 \
      --learning_rate 5e-4 \
      --num_steps_before_decay 5\
      --max_steps 10 \
      --save_freq 10 \
      --save_latest_checkpoint_only False \
      --image_aug True \
      --lora_rank 32 \
      --run_id_note franka_vla_smoke_test
```

## How OpenVLA-OFT Loads the Dataset

The training script receives:

```bash
--data_root_dir ~/Dataset/OpenVLA_TFDS
--dataset_name franka_vla
```

Important: `--data_root_dir` must point to the TFDS root folder that contains `franka_vla/1.0.0`, not directly to the `1.0.0` folder.

Inside OpenVLA-OFT, the flow is:

```text
vla-scripts/finetune.py
  -> creates RLDSDataset(...)
  -> passes cfg.data_root_dir and cfg.dataset_name

prismatic/vla/datasets/datasets.py
  -> looks up dataset_name in OXE_NAMED_MIXTURES
  -> builds the Open-X dataset mixture

prismatic/vla/datasets/rlds/oxe/materialize.py
  -> reads OXE_DATASET_CONFIGS[dataset_name]
  -> reads OXE_STANDARDIZATION_TRANSFORMS[dataset_name]
  -> constructs the RLDS/TFDS loading kwargs

prismatic/vla/datasets/rlds/dataset.py
  -> opens the TFDS dataset
  -> applies trajectory transforms
  -> applies image resize/augmentation
  -> samples frames for training
```

That is why the three registry files were edited.

## Change 1: `configs.py`

File:

```text
openvla-oft-main/prismatic/vla/datasets/rlds/oxe/configs.py
```

Added:

```python
"franka_vla": {
    "image_obs_keys": {
        "primary": "image",
        "secondary": None,
        "wrist": None,
    },
    "depth_obs_keys": {
        "primary": None,
        "secondary": None,
        "wrist": None,
    },
    "state_obs_keys": ["state"],
    "state_encoding": StateEncoding.POS_EULER,
    "action_encoding": ActionEncoding.EEF_POS,
},
```

Purpose:

This tells OpenVLA-OFT how to interpret the fields in the Franka RLDS dataset. 

Meaning:

The main camera image is stored at `observation/image`. These is no secondary or wrist camera. There are no depth images and the proprio/state vector is stored at `observation/state`. 

The state encoding is `StateEncoding.POS_EULER`. This means OpenVLA-OFT expect an 8-D state:
```text
EEF XYZ                    3 values
roll, pitch, yaw           3 values
padding                    1 value
gripper                    1 value
```

For this dataset `[x, y, z, roll, pitch, yaw, 0.0, gripper_width]`, the action encoding is `ActionEncoding.EEF_POS`. This means OpenVLA-OFT expects a 7-D end-effector delta action `[dx, dy, dz, droll, dpitch, dyaw, gripper]`.


Why this matters:

OpenVLA-OFT's `materialize.py` uses this config to decide:

```text
which RGB image fields to load
which depth fields to ignore
which proprio fields to load
how actions should be normalized
what action shape the model should train on
```

For `ActionEncoding.EEF_POS`, OpenVLA-OFT treats the first six action dimensions as normalized relative motion and the last gripper dimension as absolute.

## Change 2: `transforms.py`

File:

```text
openvla-oft-main/prismatic/vla/datasets/rlds/oxe/transforms.py
```

Added:

```python
def franka_vla_dataset_transform(trajectory):
      return trajectory
```

Registered:

```python
"franka_vla": franka_vla_dataset_transform,
```

Purpose:

Every Open-X dataset in OpenVLA-OFT has a standardization transform. Some datasets need real transformations because their raw field names or action formats do not match OpenVLA-OFT's expected structure.

For `franka_vla`, no transformation is needed because the converted RLDS dataset already has the correct fields:

```text
observation/image
observation/state
action
language_instruction
```

So the transform simply returns the trajectory unchanged. So immediately after that transform, the trajectory is still:
```text
trajectory
├── observation
│   ├── image              [T, 224, 224, 3], uint8
│   ├── state              [T, 8], float32
│   ├── eef_pose           [T, 6], float32
│   ├── gripper_width      [T], float32
│   ├── joint_position     [T, 7], float32
│   └── joint_velocity     [T, 7], float32
├── action                 [T, 7], float32
├── language_instruction   [T], string
├── reward
├── discount
├── is_first
├── is_last
├── is_terminal
└── timestamp_ns
```

Why this still matters:

OpenVLA-OFT still expects to find a transform function in `OXE_STANDARDIZATION_TRANSFORMS[dataset_name]`


Without registering `franka_vla`, the dataset lookup would fail when `materialize.py` tries to build the RLDS loading kwargs.

## Change 3: `mixtures.py`

File:

```text
openvla-oft-main/prismatic/vla/datasets/rlds/oxe/mixtures.py
```

Added:

```python
"franka_vla": [
    ("franka_vla", 1.0),
],
```

Purpose:

This registers `franka_vla` as a named Open-X dataset mixture.

OpenVLA-OFT's data loader is built around mixtures. A mixture is a list of datasets and sampling weights. Large training mixtures may contain many datasets, for example RT-X or Magic Soup. For this custom fine-tuning run, the mixture contains only one dataset:

```text
dataset: franka_vla
weight: 1.0
```

This allows the command:

```bash
--dataset_name franka_vla
```

to mean:

```text
train only on the Franka dataset, sampled with weight 1.0
```

Why this matters:

When `RLDSDataset` receives `data_mix="franka_vla"`, it checks `OXE_NAMED_MIXTURES`. Because this entry exists, the loader treats `franka_vla` as an official named mixture.

## Summary

The changes make OpenVLA-OFT aware of the custom Franka Panda dataset:

```text
configs.py
  Tells OpenVLA-OFT which fields exist and what action/state format they use.

transforms.py
  Registers a no-op standardization transform because the converted RLDS dataset is already in the expected format.

mixtures.py
  Allows --dataset_name franka_vla to select the custom dataset as a single-dataset training mixture.
```

With these edits, OpenVLA-OFT can find and load:

```text
OpenVLA_TFDS/franka_vla/1.0.0
```

and interpret each step as:

```text
image:    observation/image
state:    observation/state
action:   action
language: language_instruction
```

This is the minimum source-code integration needed for fine-tuning OpenVLA-OFT on the recorded Franka Panda RLDS dataset.