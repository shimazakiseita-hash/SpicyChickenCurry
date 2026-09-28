#!/usr/bin/env python3
"""書き出した policy.npz を MuJoCo 上で評価する (ROS ノードと同じ numpy 推論を使う).

使い方 (リポジトリのルートで):
    .venv/bin/python mujoco/eval_reach.py mujoco/runs/reach_<日時>/policy.npz            # 成功率を表示
    .venv/bin/python mujoco/eval_reach.py mujoco/runs/reach_<日時>/policy.npz --viewer   # ビューアで見る
"""

import argparse
import sys
import time
from pathlib import Path

import mujoco.viewer
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'mujoco'))
sys.path.insert(0, str(ROOT / 'ros2_ws/src/crane_x7_rl'))
from crane_x7_rl.policy import NumpyPolicy  # noqa: E402
from envs.crane_x7_reach import CraneX7ReachEnv  # noqa: E402


def run_episode(env, policy, seed, on_step=None):
    env.reset(seed=seed)
    for _ in range(policy.episode_steps):
        obs = policy.build_observation(
            env.data.qpos[env.qpos_idx], env.data.qvel[env.qvel_idx], env._ee_pos(), env.goal)
        _, _, _, _, info = env.step(policy(obs))
        if on_step:
            on_step()
    return info


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('policy', type=Path)
    parser.add_argument('--episodes', type=int, default=200)
    parser.add_argument('--viewer', action='store_true')
    args = parser.parse_args()

    policy = NumpyPolicy(args.policy)
    env = CraneX7ReachEnv()

    if args.viewer:
        with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
            episode = 0
            while viewer.is_running():
                def sync():
                    viewer.sync()
                    time.sleep(policy.control_dt)
                info = run_episode(env, policy, seed=episode, on_step=sync)
                print(f'episode {episode}: 誤差 {info["distance"] * 1000:.1f} mm')
                episode += 1
        return

    dists, contacts = [], 0
    for i in range(args.episodes):
        info = run_episode(env, policy, seed=10_000 + i)
        dists.append(info['distance'])
        contacts += info['contact']
    dists = np.array(dists) * 1000
    print(f'{args.episodes} エピソード: 成功率 (< 20 mm) {np.mean(dists < 20):.1%}, '
          f'誤差 中央値 {np.median(dists):.1f} mm / 90% 点 {np.percentile(dists, 90):.1f} mm, '
          f'最後に接触していた回数 {contacts}')


if __name__ == '__main__':
    main()
