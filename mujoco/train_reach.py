#!/usr/bin/env python3
"""到達タスクを PPO で学習する.

使い方 (リポジトリのルートで):
    .venv/bin/python mujoco/train_reach.py                 # 学習 (既定 200 万ステップ)
    .venv/bin/python mujoco/train_reach.py --steps 300000  # 短く試す

結果は mujoco/runs/reach_<日時>/ に保存される:
    model.zip          SB3 のモデル (学習の再開・評価用)
    vecnormalize.pkl   観測の正規化パラメータ
    policy.npz         ROS ノード用に書き出した方策 (numpy だけで推論できる)
"""

import argparse
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.logger import configure
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize

sys.path.insert(0, str(Path(__file__).resolve().parent))
from envs.crane_x7_reach import CraneX7ReachEnv  # noqa: E402
from export_policy import export_policy  # noqa: E402

RUNS_DIR = Path(__file__).resolve().parent / 'runs'


class EpisodeStatsCallback(BaseCallback):
    """エピソード終了時の距離・成功率をログに出す."""

    def __init__(self):
        super().__init__()
        self.final_dist = []
        self.success = []

    def _on_step(self):
        for done, info in zip(self.locals['dones'], self.locals['infos']):
            if done:
                self.final_dist.append(info['distance'])
                self.success.append(info['is_success'])
        return True

    def _on_rollout_end(self):
        if self.final_dist:
            self.logger.record('reach/final_distance', np.mean(self.final_dist))
            self.logger.record('reach/success_rate', np.mean(self.success))
            self.final_dist.clear()
            self.success.clear()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--steps', type=int, default=2_000_000)
    parser.add_argument('--envs', type=int, default=8)
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()

    run_dir = RUNS_DIR / f'reach_{datetime.now():%Y%m%d_%H%M%S}'
    run_dir.mkdir(parents=True)
    torch.set_num_threads(2)

    env = make_vec_env(CraneX7ReachEnv, n_envs=args.envs, seed=args.seed, vec_env_cls=SubprocVecEnv)
    env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.0)

    model = PPO(
        'MlpPolicy', env,
        n_steps=1024, batch_size=1024, n_epochs=10,
        learning_rate=3e-4, gamma=0.99, gae_lambda=0.95, ent_coef=0.0,
        policy_kwargs=dict(net_arch=dict(pi=[128, 128], vf=[128, 128])),
        seed=args.seed, verbose=0,
    )
    # 学習の経過は progress.csv に保存 (reach/success_rate が成功率)
    model.set_logger(configure(str(run_dir), ['stdout', 'csv']))
    print(f'学習開始: {args.steps} steps, {args.envs} envs -> {run_dir}')
    model.learn(total_timesteps=args.steps, callback=EpisodeStatsCallback(), progress_bar=False)

    model.save(run_dir / 'model.zip')
    env.save(str(run_dir / 'vecnormalize.pkl'))
    export_policy(model, env, run_dir / 'policy.npz')
    env.close()
    print(f'保存しました: {run_dir}')


if __name__ == '__main__':
    main()
