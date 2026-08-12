### Example 1
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
  --lora_dropout 0.05 \
  --run_id_note franka_vla_v1
```

### Example 2
```bash
export WANDB_MODE=offline

torchrun --standalone --nnodes 1 --nproc-per-node 1 vla-scripts/finetune.py \
  --vla_path openvla/openvla-7b \
  --data_root_dir ~/Dataset/Tensorflow \
  --dataset_name franka_vla \
  --run_root_dir ./runs \
  --use_l1_regression True \
  --use_diffusion False \
  --use_film True \
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
  --lora_dropout 0.05 \
  --run_id_note franka_vla_v1
```
