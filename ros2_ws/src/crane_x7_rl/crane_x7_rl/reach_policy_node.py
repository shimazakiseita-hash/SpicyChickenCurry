"""MuJoCo で学習した到達タスクの方策を、Gazebo (または実機) の CRANE-X7 で動かすノード.

1. アームを初期姿勢 (全関節 0 = 直立) に戻す
2. 目標位置をランダムに決め (学習時と同じ範囲)、Gazebo と RViz に目標の球を表示する
3. 方策を control_dt ごとに実行して関節の目標角度を送る (学習時の 1 エピソード分)
4. 1 に戻る

観測・行動の定義は mujoco/envs/crane_x7_reach.py と同じ. 必要な設定値は policy.npz に入っている.
"""

import subprocess

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

from crane_x7_rl.policy import NumpyPolicy

BASE_FRAME = 'base_link'
GOAL_MODEL_NAME = 'reach_goal'
GOAL_SDF = (
    '<sdf version="1.9"><model name="{name}"><static>true</static><link name="link">'
    '<visual name="visual"><geometry><sphere><radius>0.015</radius></sphere></geometry>'
    '<material><ambient>1 0.8 0 1</ambient><diffuse>1 0.8 0 1</diffuse></material>'
    '</visual></link></model></sdf>'
)


def to_duration(seconds):
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int((seconds - sec) * 1e9))


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class ReachPolicyNode(Node):

    def __init__(self):
        super().__init__('reach_policy')
        self.declare_parameter('policy_path', '')
        self.declare_parameter('home_duration', 2.0)
        self.declare_parameter('show_in_gazebo', True)
        self.declare_parameter('gazebo_world', 'default')
        # Gazebo のワールド座標で見た base_link の高さ (crane_x7_with_table.launch.py の -z)
        self.declare_parameter('base_z_in_gazebo', 1.015)

        policy_path = self.get_parameter('policy_path').value
        if not policy_path:
            raise RuntimeError('policy_path パラメータに policy.npz のパスを指定してください')
        self.policy = NumpyPolicy(policy_path)
        self.home_duration = self.get_parameter('home_duration').value
        self.show_in_gazebo = self.get_parameter('show_in_gazebo').value
        self.gazebo_world = self.get_parameter('gazebo_world').value
        self.base_z_in_gazebo = self.get_parameter('base_z_in_gazebo').value

        self.rng = np.random.default_rng()
        self.joint_pos = None
        self.joint_vel = None
        self.goal = None
        self.target = None
        self.phase = 'home'
        self.phase_steps = 0
        self.home_steps = int(round(self.home_duration / self.policy.control_dt)) + 10

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/crane_x7_arm_controller/joint_trajectory', 10)
        self.marker_pub = self.create_publisher(Marker, 'reach_goal_marker', 10)

        if self.show_in_gazebo:
            self.gz_service('create', 'gz.msgs.EntityFactory',
                            f'sdf: \'{GOAL_SDF.format(name=GOAL_MODEL_NAME)}\' '
                            f'name: "{GOAL_MODEL_NAME}" pose: {{position: {{z: -1}}}}')

        self.create_timer(self.policy.control_dt, self.on_timer)
        self.get_logger().info(f'policy を読み込みました: {policy_path}')

    def gz_service(self, service, reqtype, req):
        # 返事を待つと制御周期が乱れるので、投げっぱなしにする
        subprocess.Popen(
            ['gz', 'service', '-s', f'/world/{self.gazebo_world}/{service}',
             '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean', '--timeout', '2000',
             '--req', req],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def on_joint_states(self, msg):
        index = {name: i for i, name in enumerate(msg.name)}
        if not all(name in index for name in self.policy.joint_names):
            return
        ids = [index[name] for name in self.policy.joint_names]
        self.joint_pos = np.array([msg.position[i] for i in ids])
        self.joint_vel = np.array([msg.velocity[i] for i in ids]) if msg.velocity else np.zeros(len(ids))

    def ee_position(self):
        tf = self.tf_buffer.lookup_transform(BASE_FRAME, self.policy.ee_body, rclpy.time.Time())
        t = tf.transform.translation
        return np.array([t.x, t.y, t.z]) + quat_to_matrix(tf.transform.rotation) @ self.policy.ee_offset

    def send_target(self, positions, duration):
        traj = JointTrajectory()
        traj.joint_names = self.policy.joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = to_duration(duration)
        traj.points.append(point)
        self.traj_pub.publish(traj)

    def show_goal(self):
        marker = Marker()
        marker.header.frame_id = BASE_FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'reach_goal'
        marker.type = Marker.SPHERE
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, self.goal)
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.03
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.8, 0.0, 0.8
        self.marker_pub.publish(marker)

        if self.show_in_gazebo:
            x, y, z = self.goal
            self.gz_service('set_pose', 'gz.msgs.Pose',
                            f'name: "{GOAL_MODEL_NAME}" '
                            f'position: {{x: {x}, y: {y}, z: {z + self.base_z_in_gazebo}}}')

    def on_timer(self):
        if self.joint_pos is None:
            return
        self.phase_steps += 1

        if self.phase == 'home':
            if self.phase_steps == 1:
                self.send_target(np.zeros(len(self.policy.joint_names)), self.home_duration)
            if self.phase_steps >= self.home_steps:
                self.goal = self.rng.uniform(self.policy.goal_low, self.policy.goal_high)
                self.target = self.joint_pos.copy()
                self.show_goal()
                self.phase, self.phase_steps = 'reach', 0
                self.get_logger().info(f'目標: {np.round(self.goal, 3)}')
            return

        try:
            ee = self.ee_position()
        except TransformException as e:
            self.get_logger().warn(f'手先位置を取得できません: {e}', throttle_duration_sec=2.0)
            return

        obs = self.policy.build_observation(self.joint_pos, self.joint_vel, ee, self.goal)
        action = self.policy(obs)
        self.target = np.clip(
            self.target + action * self.policy.max_delta, self.policy.joint_low, self.policy.joint_high)
        self.send_target(self.target, self.policy.control_dt)

        if self.phase_steps >= self.policy.episode_steps:
            dist = np.linalg.norm(self.goal - ee)
            self.get_logger().info(f'到達誤差: {dist * 1000:.1f} mm')
            self.phase, self.phase_steps = 'home', 0


def main():
    rclpy.init()
    node = ReachPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
