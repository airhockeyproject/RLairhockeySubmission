import myenv
import gymnasium as gym
from gymnasium.wrappers import TimeLimit
import cv2
import os
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor
from stable_baselines3.common.callbacks import CheckpointCallback
from tqdm import tqdm
import argparse
import torch

is_cuda_avaliable = torch.cuda.is_available()


def parse_args():
    parser = argparse.ArgumentParser(description="強化学習のコード")
    # 学習設定
    parser.add_argument("--train", action="store_true", help="学習するかどうか")
    parser.add_argument("--experiment_name", type=str, required=True, help="実験名")
    parser.add_argument(
        "--total_timesteps", type=int, default=100000, help="学習ロールアウト数"
    )
    parser.add_argument(
        "--max_episode_steps", type=int, default=300, help="打ち切りステップ数"
    )
    parser.add_argument("--log_interval", type=int, default=4000, help="ログ出力の間隔")

    # モデル関連
    parser.add_argument(
        "--model_save_dir", type=str, required=True, help="モデルの保存ディレクトリ"
    )
    parser.add_argument(
        "--model_path", type=str, default=None, help="再開するモデルの重みのパス"
    )
    parser.add_argument("--save_freq", type=int, default=20000, help="モデルの保存頻度")

    # 環境設定
    parser.add_argument("--num_envs", type=int, default=8, help="環境並列数")
    return parser.parse_args()


def make_env(max_episode_steps):
    def _init():
        env = gym.make("AirHockey-v0")
        return TimeLimit(env, max_episode_steps)

    return _init


def render(images):
    for img in images:
        # 画像をuint8のBGRに変換（OpenCV用）
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        cv2.imshow("Video", img_bgr)
        if cv2.waitKey(20) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()


def load_model(env, model_path=None):
    device = "cuda" if is_cuda_avaliable else "cpu"
    if model_path is None:
        model = SAC("MlpPolicy", env, verbose=2, device=device)
    else:
        model = SAC.load(model_path, env, verbose=2, device=device)
    return model


def evaluate(model, max_episode_steps):
    env = make_env(max_episode_steps)()
    obs, _ = env.reset()
    images = []
    for _ in tqdm(range(1000)):
        action, _ = model.predict(obs, deterministic=True)
        obs, _, terminated, truncated, _ = env.step(action)
        images.append(env.render())
        if terminated or truncated:
            env.reset()
    render(images)


def main():
    args = parse_args()
    env = VecMonitor(
        SubprocVecEnv([make_env(args.max_episode_steps) for _ in range(args.num_envs)]),
        "logs/monitor",
    )
    model = load_model(env, args.model_path)

    if args.train:
        checkpoint_callback = CheckpointCallback(
            save_freq=args.save_freq,
            save_path=args.model_save_dir,
            name_prefix=args.experiment_name,
            save_replay_buffer=True,
            save_vecnormalize=True,
        )
        model.learn(
            total_timesteps=args.total_timesteps,
            log_interval=args.log_interval,
            callback=checkpoint_callback,
        )
        save_path = os.path.join(args.model_save_dir, args.experiment_name + "_final")
        model.save(save_path)

    evaluate(model, args.max_episode_steps)


if __name__ == "__main__":
    main()
