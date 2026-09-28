#!/usr/bin/env python3
"""学習済みモデルを、ROS ノードが numpy だけで使える policy.npz に書き出す.

使い方 (学習スクリプトの最後でも自動で呼ばれる):
    .venv/bin/python mujoco/export_policy.py mujoco/runs/reach_<日時>

policy.npz には方策の重みに加えて、ROS 側で観測・行動を再現するための
設定 (関節名・可動範囲・制御周期・手先オフセットなど) も入れておく.
"""

import argparse
import pickle
import sys
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize

sys.path.insert(0, str(Path(__file__).resolve().parent))
from envs import crane_x7_reach  # noqa: E402


def export_policy(model: PPO, vec_normalize: VecNormalize, path: Path):
    policy = model.policy
    arrays = {}
    layers = [m for m in policy.mlp_extractor.policy_net if isinstance(m, torch.nn.Linear)]
    for i, layer in enumerate(layers):
        arrays[f'w{i}'] = layer.weight.detach().cpu().numpy()
        arrays[f'b{i}'] = layer.bias.detach().cpu().numpy()
    arrays['w_out'] = policy.action_net.weight.detach().cpu().numpy()
    arrays['b_out'] = policy.action_net.bias.detach().cpu().numpy()

    arrays['obs_mean'] = vec_normalize.obs_rms.mean
    arrays['obs_var'] = vec_normalize.obs_rms.var
    arrays['obs_clip'] = np.array(vec_normalize.clip_obs)
    arrays['obs_eps'] = np.array(vec_normalize.epsilon)

    env = crane_x7_reach.CraneX7ReachEnv()
    site = env.model.site('ee_site')
    arrays['joint_names'] = np.array(crane_x7_reach.ARM_JOINTS)
    arrays['joint_low'] = env.ctrl_low
    arrays['joint_high'] = env.ctrl_high
    arrays['ee_body'] = np.array(env.model.body(site.bodyid[0]).name)
    arrays['ee_offset'] = site.pos.copy()
    arrays['control_dt'] = np.array(crane_x7_reach.CONTROL_DT)
    arrays['max_delta'] = np.array(crane_x7_reach.MAX_DELTA)
    arrays['vel_obs_scale'] = np.array(crane_x7_reach.VEL_OBS_SCALE)
    arrays['episode_steps'] = np.array(crane_x7_reach.EPISODE_STEPS)
    arrays['goal_low'] = crane_x7_reach.GOAL_LOW
    arrays['goal_high'] = crane_x7_reach.GOAL_HIGH
    env.close()

    np.savez(path, **arrays)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dir', type=Path)
    args = parser.parse_args()

    model = PPO.load(args.run_dir / 'model.zip', device='cpu')
    # 正規化の統計量だけが必要なので、環境を作らずに pickle から直接読む
    with open(args.run_dir / 'vecnormalize.pkl', 'rb') as f:
        vec_normalize = pickle.load(f)
    export_policy(model, vec_normalize, args.run_dir / 'policy.npz')
    print(f'wrote {args.run_dir / "policy.npz"}')


if __name__ == '__main__':
    main()
