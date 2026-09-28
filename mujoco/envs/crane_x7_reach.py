"""CRANE-X7 の到達タスク: 手先 (ee_site) を目標位置に近づける.

ROS 側 (Gazebo) でも同じ観測を作れるように、すべて base_link 座標系・関節角ベースで定義している.
観測・行動の定義を変えたら ros2_ws/src/crane_x7_rl/crane_x7_rl/reach_policy_node.py も合わせること.
"""

from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np

MODEL_PATH = Path(__file__).resolve().parents[1] / 'models/crane_x7/scene.xml'

ARM_JOINTS = [
    'crane_x7_shoulder_fixed_part_pan_joint',
    'crane_x7_shoulder_revolute_part_tilt_joint',
    'crane_x7_upper_arm_revolute_part_twist_joint',
    'crane_x7_upper_arm_revolute_part_rotate_joint',
    'crane_x7_lower_arm_fixed_part_joint',
    'crane_x7_lower_arm_revolute_part_joint',
    'crane_x7_wrist_joint',
]

CONTROL_DT = 0.05          # 方策の実行周期 [s] (20 Hz)
MAX_DELTA = 0.05           # 1 ステップで動かせる目標角度の最大変化量 [rad]
EPISODE_STEPS = 100        # 5 秒
SUCCESS_DIST = 0.02        # 成功とみなす距離 [m]
VEL_OBS_SCALE = 0.1        # 関節速度を観測に入れるときの縮尺

# 目標位置をサンプリングする範囲 (base_link 座標系) [m]
GOAL_LOW = np.array([0.15, -0.25, 0.05])
GOAL_HIGH = np.array([0.40, 0.25, 0.45])

INIT_NOISE = 0.1           # 初期姿勢 (全関節 0 = 直立) に加えるノイズ [rad]


def build_observation(qpos, qvel, ee_pos, goal):
    """観測ベクトル (23 次元). ROS ノードからも同じ式で計算する."""
    return np.concatenate([
        qpos,                       # 7: 関節角 [rad]
        qvel * VEL_OBS_SCALE,       # 7: 関節速度
        ee_pos,                     # 3: 手先位置 [m]
        goal,                       # 3: 目標位置 [m]
        goal - ee_pos,              # 3: 手先→目標
    ]).astype(np.float32)


class CraneX7ReachEnv(gym.Env):
    metadata = {'render_modes': ['rgb_array'], 'render_fps': int(1 / CONTROL_DT)}

    def __init__(self, render_mode=None):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        self.frame_skip = round(CONTROL_DT / self.model.opt.timestep)
        self.render_mode = render_mode
        self._renderer = None

        joint_ids = [self.model.joint(n).id for n in ARM_JOINTS]
        self.qpos_idx = self.model.jnt_qposadr[joint_ids]
        self.qvel_idx = self.model.jnt_dofadr[joint_ids]
        self.act_idx = np.array([self.model.actuator(n).id for n in ARM_JOINTS])
        self.ctrl_low = self.model.actuator_ctrlrange[self.act_idx, 0]
        self.ctrl_high = self.model.actuator_ctrlrange[self.act_idx, 1]
        self.ee_site = self.model.site('ee_site').id
        self.goal_mocap = self.model.body('goal').mocapid[0]

        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(len(ARM_JOINTS),), dtype=np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(23,), dtype=np.float32)

        self.goal = np.zeros(3)
        self.target = np.zeros(len(ARM_JOINTS))
        self.steps = 0

    def _ee_pos(self):
        return self.data.site_xpos[self.ee_site].copy()

    def _obs(self):
        return build_observation(
            self.data.qpos[self.qpos_idx], self.data.qvel[self.qvel_idx],
            self._ee_pos(), self.goal)

    def _robot_in_contact(self):
        # base_link は world に固定されていて床との接触は計算されないので、
        # 接触が 1 つでもあれば「どこかにぶつかっている」
        return self.data.ncon > 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)

        q0 = self.np_random.uniform(-INIT_NOISE, INIT_NOISE, size=len(ARM_JOINTS))
        q0 = np.clip(q0, self.ctrl_low, self.ctrl_high)
        self.data.qpos[self.qpos_idx] = q0
        self.target = q0.copy()
        self.data.ctrl[self.act_idx] = self.target

        if options and 'goal' in options:
            self.goal = np.asarray(options['goal'], dtype=float)
        else:
            self.goal = self.np_random.uniform(GOAL_LOW, GOAL_HIGH)
        self.data.mocap_pos[self.goal_mocap] = self.goal

        mujoco.mj_forward(self.model, self.data)
        self.steps = 0
        return self._obs(), {}

    def step(self, action):
        action = np.clip(action, -1.0, 1.0)
        self.target = np.clip(self.target + action * MAX_DELTA, self.ctrl_low, self.ctrl_high)
        self.data.ctrl[self.act_idx] = self.target
        mujoco.mj_step(self.model, self.data, nstep=self.frame_skip)
        self.steps += 1

        dist = float(np.linalg.norm(self.goal - self._ee_pos()))
        contact = self._robot_in_contact()
        reward = -dist - 0.01 * float(np.square(action).sum()) - (1.0 if contact else 0.0)

        truncated = self.steps >= EPISODE_STEPS
        info = {'distance': dist, 'is_success': dist < SUCCESS_DIST, 'contact': contact}
        return self._obs(), reward, False, truncated, info

    def render(self):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, 480, 640)
            self._camera = mujoco.MjvCamera()
            self._camera.lookat[:] = [0.2, 0.0, 0.25]
            self._camera.distance = 1.3
            self._camera.azimuth = 135
            self._camera.elevation = -25
        self._renderer.update_scene(self.data, self._camera)
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
