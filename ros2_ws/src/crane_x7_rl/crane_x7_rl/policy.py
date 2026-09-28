"""mujoco/export_policy.py が書き出した policy.npz を numpy だけで実行する."""

import numpy as np


class NumpyPolicy:

    def __init__(self, path):
        data = np.load(path)
        self.layers = []
        i = 0
        while f'w{i}' in data:
            self.layers.append((data[f'w{i}'], data[f'b{i}']))
            i += 1
        self.w_out = data['w_out']
        self.b_out = data['b_out']
        self.obs_mean = data['obs_mean']
        self.obs_std = np.sqrt(data['obs_var'] + float(data['obs_eps']))
        self.obs_clip = float(data['obs_clip'])

        self.joint_names = [str(n) for n in data['joint_names']]
        self.joint_low = data['joint_low']
        self.joint_high = data['joint_high']
        self.ee_body = str(data['ee_body'])
        self.ee_offset = data['ee_offset']
        self.control_dt = float(data['control_dt'])
        self.max_delta = float(data['max_delta'])
        self.vel_obs_scale = float(data['vel_obs_scale'])
        self.episode_steps = int(data['episode_steps'])
        self.goal_low = data['goal_low']
        self.goal_high = data['goal_high']

    def build_observation(self, qpos, qvel, ee_pos, goal):
        # mujoco/envs/crane_x7_reach.py の build_observation と同じ並び
        return np.concatenate([
            qpos,
            qvel * self.vel_obs_scale,
            ee_pos,
            goal,
            goal - ee_pos,
        ])

    def __call__(self, obs):
        """学習時と同じ正規化をかけ、決定的な行動 ([-1, 1] に丸めたもの) を返す."""
        x = np.clip((obs - self.obs_mean) / self.obs_std, -self.obs_clip, self.obs_clip)
        for w, b in self.layers:
            x = np.tanh(w @ x + b)
        return np.clip(self.w_out @ x + self.b_out, -1.0, 1.0)
