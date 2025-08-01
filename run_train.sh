python train.py \
    --train \
    --experiment_name sac \
    --model_save_dir models \
    --total_timesteps 100000 \
    --log_interval 1000 \
    --num_envs 10 \
    --max_episode_steps 100 \
    --save_freq 50000 \
