"""ボールをつかんで目標位置へ置く・押し出すアクションサーバ (MoveIt / moveit_py を使う).

アクション: /boccia/move_ball (boccia_interfaces/action/MoveBall)

動作の流れ (MODE_PLACE):
  check     : 動き出す前に、つかむ・置く (・押す) ときの手が他のボールなどに当たらない
              手首の向きを決める. 見つからなければ何もせずに中止する
  approach  : グリッパーを開き、ボールの真上 (approach_height) へ移動      … 自由な移動
  grasp     : 真下へ下ろして (grasp_height) グリッパーを閉じる             … 短い移動
  lift      : 真上へ持ち上げる (lift_height)                               … 短い移動
  transport : 目標位置の真上へ移動                                         … 自由な移動
  release   : 下ろしてグリッパーを開く                                     … 短い移動
  retreat   : 真上へ逃げて、home 姿勢に戻る                                 … 短い移動 + 自由な移動

自由な移動: OMPL で、障害物 (台) を避ける経路を探す.
短い移動:   目標の手先姿勢になる関節角を「今の関節角」から少しずつ探し (solve_ik_near)、
            Pilz PTP で各関節をまっすぐ動かす. 近い関節角どうしなので、手先はほぼ直線に動く.
            手先が真下を向く姿勢は特異姿勢 (手首の軸がそろう) に近く、普通の逆運動学や
            Pilz LIN だと遠い関節角に飛んで失敗することがあるため、この方法にしている.

MODE_PUSH は、目標の push_distance 手前にボールを置いたあと (place と同じ動作)、
グリッパーを閉じてボールの後ろ (push_standoff) に手を下ろし、目標方向へ直線で押し出す.
押す向きは「ロボットの根元 → 目標」の向き (ロボットから遠ざかる向き).

他のボール (goal.obstacles) は MoveIt に球の障害物として教える. 動かすボールは、つかむまでは
障害物、つかんでから離すまでは「手に付いた物」として扱う (運ぶ途中のボールも他に当てない).

ボールの位置 (goal.ball.position) は planning_frame (base_link) の座標で渡すこと.
目標 (goal.target) は goal.target_frame の座標 (空なら planning_frame).

起動には MoveIt の設定 (robot_description など) が必要なので、
boccia_bringup/launch/manipulation.launch.py から起動すること.
"""

import numpy as np
import rclpy
import yaml
from boccia_interfaces.action import MoveBall
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener

# グリッパーを真下に向ける姿勢 (roll, pitch) [deg]. yaw (真下を向いたままの回転) は場面ごとに選ぶ.
# 指は gripper_base_link の x 軸方向に開くので、yaw [deg] のとき指が開く向きは
# base_link で (cos yaw, sin yaw). 既定の -90° は crane_x7_examples_py の pick_and_place と同じ
DOWNWARD_ROLL_PITCH_DEG = (-180.0, 0.0)
DEFAULT_YAW_DEG = -90.0
# 手首の向きを決めるとき、これより遠いボールは気にしない [m]
YAW_SEARCH_RADIUS = 0.12
# MoveIt に教えるボールの球を、本物よりこれだけ大きくする [m] (余裕)
BALL_COLLISION_MARGIN = 0.005
# 手首の向きの候補を、最初に選んだ向きからこの刻みで試す [deg]
YAW_SEARCH_STEP_DEG = 15.0
# グリッパーの中で、運ぶボールに触れてよいリンク
GRIPPER_LINKS = ['crane_x7_gripper_base_link', 'crane_x7_gripper_finger_a_link',
                 'crane_x7_gripper_finger_b_link']
# 「短い移動」で許す関節角の変化の最大値 [rad]. これより大きいと別の姿勢に飛んだとみなす
MAX_SHORT_MOVE_JOINT_CHANGE = 0.8
# solve_ik_near の設定: 繰り返し回数、許容誤差 [m] [rad]、1 回の最大の動き、減衰
IK_MAX_ITERATIONS = 300
IK_POS_TOLERANCE = 0.0005
IK_ROT_TOLERANCE = 0.005
IK_MAX_STEP = 0.02
IK_DAMPING = 0.05
# 関節の可動範囲の端から、これだけ内側までしか使わない [rad]
JOINT_LIMIT_MARGIN = 0.02


class MotionError(Exception):
    """計画または実行に失敗した."""


def normalize_yaw_deg(yaw):
    """指は左右対称なので yaw は 180° ごとに同じ. (-180, 0] にそろえる (既定の -90° に近い側)."""
    return (yaw % 180.0) - 180.0 if yaw % 180.0 != 0 else 0.0


def choose_yaw_deg(xy, obstacles):
    """xy の近くにあるボールに指が当たらないよう、指が開く向きを「一番近いボールの向き」と直角にする."""
    nearest, best = None, YAW_SEARCH_RADIUS
    for ball in obstacles:
        d = np.hypot(ball.position.x - xy[0], ball.position.y - xy[1])
        if d < best:
            nearest, best = ball, d
    if nearest is None:
        return DEFAULT_YAW_DEG
    angle = np.degrees(np.arctan2(nearest.position.y - xy[1], nearest.position.x - xy[0]))
    return normalize_yaw_deg(angle + 90.0)


class MoveBallServer(Node):

    def __init__(self):
        super().__init__('move_ball_server')
        defaults = [
            ('arm_group', 'arm'), ('gripper_group', 'gripper'),
            ('ee_link', 'crane_x7_gripper_base_link'), ('planning_frame', 'base_link'),
            ('approach_height', 0.13), ('grasp_height', 0.08), ('lift_height', 0.08),
            ('gripper_open', 0.9), ('gripper_close', 0.43), ('gripper_holding', 0.52),
            ('release_clearance', -0.008),
            ('push_standoff', 0.02), ('push_distance', 0.05), ('push_height', 0.08),
            ('push_contact_offset', 0.048),
            ('velocity_scaling', 0.3), ('acceleration_scaling', 0.3),
            ('gripper_velocity_scaling', 0.1), ('push_velocity_scaling', 0.1),
            ('table_surface_z', 0.0), ('return_home', True),
            ('court_config', ''), ('keepout_height', 0.07),
        ]
        for name, default in defaults:
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name, _ in defaults}

        if not self.p['court_config']:
            raise RuntimeError('court_config パラメータに court.yaml のパスを指定してください')
        with open(self.p['court_config']) as f:
            self.court = yaml.safe_load(f)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.moveit = MoveItPy(node_name='move_ball_moveit')
        self.arm = self.moveit.get_planning_component(self.p['arm_group'])
        self.gripper = self.moveit.get_planning_component(self.p['gripper_group'])
        # 逆運動学で関節の可動範囲を超えないように、範囲を控えておく (少し内側にする)
        bounds = self.moveit.get_robot_model().get_joint_model_group(
            self.p['arm_group']).active_joint_model_bounds
        self.joint_low = np.array([b[0].min_position for b in bounds]) + JOINT_LIMIT_MARGIN
        self.joint_high = np.array([b[0].max_position for b in bounds]) - JOINT_LIMIT_MARGIN
        self.yaw = DEFAULT_YAW_DEG   # 今の手首の向き [deg] (ee_pose / solve_ik_near が使う)
        self.attached = None         # 手に付けているボールの名前
        self.free_params = self.plan_params('ompl_rrtc')    # 自由な移動
        self.short_params = self.plan_params('pilz_ptp')    # 短い移動 (関節空間で直線)
        self.push_params = self.plan_params('pilz_ptp')     # 押し出し (速さを別に決める)
        self.push_params.max_velocity_scaling_factor = self.p['push_velocity_scaling']
        self.push_params.max_acceleration_scaling_factor = self.p['push_velocity_scaling']
        self.gripper_params = self.plan_params('ompl_rrtc')  # グリッパー (ゆっくり)
        self.gripper_params.max_velocity_scaling_factor = self.p['gripper_velocity_scaling']
        self.gripper_params.max_acceleration_scaling_factor = self.p['gripper_velocity_scaling']
        self.add_table_to_planning_scene()

        self.server = ActionServer(
            self, MoveBall, '/boccia/move_ball', self.execute,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info('MoveBall アクションサーバを起動しました')

    # ---- MoveIt の準備 ----

    def plan_params(self, namespace):
        params = PlanRequestParameters(self.moveit, namespace)
        params.max_velocity_scaling_factor = self.p['velocity_scaling']
        params.max_acceleration_scaling_factor = self.p['acceleration_scaling']
        return params

    def set_box(self, name, center, size, add=True):
        """MoveIt の planning scene に箱の障害物を置く (add=False なら取り除く)."""
        obj = CollisionObject()
        obj.id = name
        obj.header.frame_id = self.p['planning_frame']
        if add:
            obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size)))
            pose = PoseStamped().pose
            pose.position.x, pose.position.y, pose.position.z = center
            pose.orientation.w = 1.0
            obj.primitive_poses.append(pose)
            obj.operation = CollisionObject.ADD
        else:
            obj.operation = CollisionObject.REMOVE
        with self.moveit.get_planning_scene_monitor().read_write() as scene:
            scene.apply_collision_object(obj)
            scene.current_state.update()

    def add_table_to_planning_scene(self):
        """台の上面を障害物として MoveIt に教える (台を突き抜ける計画をさせない)."""
        # 上面を table_surface_z より少し下にする. ロボットの根元と重なって「衝突中」にならないように、
        # また置くときにボールを台に押し付ける (release_clearance) 分、手に付けたボールがめり込めるように
        z = self.p['table_surface_z'] - 0.05 - 0.015
        self.set_box('boccia_table', (0.0, 0.0, z), (2.0, 2.0, 0.1))

    def set_court_keepout(self, add):
        """コートの上の低い空間 (ボールがある高さ) を立ち入り禁止にする / 解除する.

        MoveIt はボールの位置を知らないので、自由な移動 (OMPL) の経路がボールの上を低く通ると
        ボールを弾いてしまう. 自由な移動のときだけこの箱を置き、ボールの上を高く通らせる.
        """
        c = self.court['court']
        if c['yaw'] != 0.0:
            raise MotionError('court.yaml の yaw が 0 以外の場合はまだ対応していません')
        h = self.p['keepout_height']
        center = (c['origin']['x'] + c['length'] / 2, c['origin']['y'] + c['width'] / 2,
                  c['surface_z'] + h / 2)
        self.set_box('boccia_court_keepout', center, (c['length'], c['width'], h), add)

    def set_ball(self, name, position, diameter, add=True):
        """ボールを球の障害物として MoveIt に教える (add=False なら取り除く)."""
        obj = CollisionObject()
        obj.id = name
        obj.header.frame_id = self.p['planning_frame']
        if add:
            radius = diameter / 2 + BALL_COLLISION_MARGIN
            obj.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[radius]))
            pose = PoseStamped().pose
            pose.position.x, pose.position.y, pose.position.z = position.x, position.y, position.z
            pose.orientation.w = 1.0
            obj.primitive_poses.append(pose)
            obj.operation = CollisionObject.ADD
        else:
            obj.operation = CollisionObject.REMOVE
        with self.moveit.get_planning_scene_monitor().read_write() as scene:
            scene.apply_collision_object(obj)
            scene.current_state.update()

    def attach_ball(self, name, position, diameter, attach=True):
        """つかんだボールを手に付いた物として扱う (運ぶ途中で他のボールに当てないため)."""
        aco = AttachedCollisionObject()
        aco.link_name = self.p['ee_link']
        aco.touch_links = GRIPPER_LINKS
        aco.object.id = name
        aco.object.header.frame_id = self.p['planning_frame']
        if attach:
            # 運ぶボールは台に接したままつかむので、余裕を付けると「台に当たっている」扱いになる.
            # 本物の大きさにする
            radius = diameter / 2
            aco.object.primitives.append(
                SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[radius]))
            pose = PoseStamped().pose
            pose.position.x, pose.position.y, pose.position.z = position.x, position.y, position.z
            pose.orientation.w = 1.0
            aco.object.primitive_poses.append(pose)
            aco.object.operation = CollisionObject.ADD
        else:
            aco.object.operation = CollisionObject.REMOVE
        with self.moveit.get_planning_scene_monitor().read_write() as scene:
            scene.process_attached_collision_object(aco)
            scene.current_state.update()
        self.attached = name if attach else None
        if not attach:
            # 手から外すと、MoveIt はその物を「置いた場所にある障害物」として残すので、それも消す
            # (残すと、ボールを離した直後の指が「ボールに当たっている」扱いになって動けない)
            self.set_ball(name, None, 0.0, add=False)

    def clear_balls(self, names):
        if self.attached is not None:
            self.attach_ball(self.attached, None, 0.0, attach=False)
        for name in names:
            self.set_ball(name, None, 0.0, add=False)

    # ---- 動作の部品 ----

    def hand_rotation(self):
        return Rotation.from_euler('xyz', (*DOWNWARD_ROLL_PITCH_DEG, self.yaw), degrees=True)

    def ee_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = self.p['planning_frame']
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        q = self.hand_rotation().as_quat()
        pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
        pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
        return pose

    def run(self, component, params, what, group):
        result = component.plan(single_plan_parameters=params)
        if not result:
            raise MotionError(f'{what} の計画に失敗しました')
        # Pilz などは経路の途中の衝突を確かめないことがあるので、実行前に経路上の姿勢を全部確かめる
        # (scene.is_path_valid は moveit_py の型の都合で使えないので、1 つずつ is_state_colliding で見る)
        with self.moveit.get_planning_scene_monitor().read_only() as scene:
            for state, _time in result.trajectory:   # (姿勢, 時刻) の組
                if scene.is_state_colliding(robot_state=state, joint_model_group_name=group,
                                            verbose=True):
                    raise MotionError(f'{what} の経路が他のボールなどに当たるので中止しました')
        if not self.moveit.execute(result.trajectory, controllers=[]):
            raise MotionError(f'{what} の実行に失敗しました')

    def move_hand(self, x, y, z, short, what, params=None):
        self.arm.set_start_state_to_current_state()
        pose = self.ee_pose(x, y, z)
        if not short:
            self.arm.set_goal_state(pose_stamped_msg=pose, pose_link=self.p['ee_link'])
            self.run_free(what)
            return

        with self.moveit.get_planning_scene_monitor().read_only() as scene:
            start = np.asarray(scene.current_state.get_joint_group_positions(self.p['arm_group']))
        state = RobotState(self.moveit.get_robot_model())
        goal = self.solve_ik_near(state, start, np.array([x, y, z]))
        if goal is None:
            raise MotionError(f'{what} の関節角が見つかりません')
        jump = float(np.max(np.abs(goal - start)))
        if jump > MAX_SHORT_MOVE_JOINT_CHANGE:
            raise MotionError(f'{what} で関節が大きく動きすぎます ({jump:.2f} rad)')
        self.arm.set_goal_state(robot_state=state)
        self.run(self.arm, params or self.short_params, what, self.p['arm_group'])

    def solve_ik_near(self, state, start, target_pos):
        """手先を target_pos・真下向きにする関節角を、start から少しずつ動かして探す.

        減衰最小二乗法 (dq = J^T (J J^T + λ^2 I)^-1 e) を繰り返す. 関節角の変化が最小になる
        向きに動くので、今の姿勢に近い解が見つかる. 見つかれば state をその関節角にして返す.
        """
        group, ee = self.p['arm_group'], self.p['ee_link']
        target_rot = self.hand_rotation()
        q = start.copy()
        for _ in range(IK_MAX_ITERATIONS):
            state.set_joint_group_positions(group, q)
            state.update()
            tf = state.get_global_link_transform(ee)
            pos_err = target_pos - tf[:3, 3]
            rot_err = (target_rot * Rotation.from_matrix(tf[:3, :3]).inv()).as_rotvec()
            if np.linalg.norm(pos_err) < IK_POS_TOLERANCE and np.linalg.norm(rot_err) < IK_ROT_TOLERANCE:
                return q
            err = np.concatenate([pos_err, rot_err])
            err *= min(1.0, IK_MAX_STEP / np.linalg.norm(err))   # 1 回に大きく動きすぎない
            jac = state.get_jacobian(group, np.zeros(3))          # 6 x 7 (上 3 行が並進、下 3 行が回転)
            q = q + jac.T @ np.linalg.solve(jac @ jac.T + IK_DAMPING**2 * np.eye(6), err)
            q = np.clip(q, self.joint_low, self.joint_high)
        return None

    def hand_collides(self, xy, z, yaw, finger_angle):
        """手を (xy, z)・向き yaw に置き、指を finger_angle にしたとき、何かに当たるか (動かさずに確かめる)."""
        with self.moveit.get_planning_scene_monitor().read_only() as scene:
            start = np.asarray(scene.current_state.get_joint_group_positions(self.p['arm_group']))
        state = RobotState(self.moveit.get_robot_model())
        saved, self.yaw = self.yaw, yaw
        try:
            q = self.solve_ik_near(state, start, np.array([xy[0], xy[1], z]))
        finally:
            self.yaw = saved
        if q is None:
            return True   # その姿勢にできない
        state.set_joint_group_positions(self.p['gripper_group'], [finger_angle])
        state.update()
        with self.moveit.get_planning_scene_monitor().read_only() as scene:
            return any(scene.is_state_colliding(robot_state=state, joint_model_group_name=g,
                                                verbose=False)
                       for g in (self.p['arm_group'], self.p['gripper_group']))

    def find_free_yaw(self, checks, preferred, what):
        """checks = [(xy, z, 指の角度), ...] のどれでも何にも当たらない手首の向きを、preferred に近い順に探す."""
        offsets = [0.0]
        for k in range(1, int(90 / YAW_SEARCH_STEP_DEG) + 1):
            offsets += [k * YAW_SEARCH_STEP_DEG, -k * YAW_SEARCH_STEP_DEG]
        for offset in offsets:
            yaw = normalize_yaw_deg(preferred + offset)
            if not any(self.hand_collides(xy, z, yaw, finger) for xy, z, finger in checks):
                return yaw
        raise MotionError(f'{what}: どの手首の向きでも指が他のボールなどに当たるので中止しました')

    def run_free(self, what):
        """自由な移動 (OMPL). コートの上の低い空間は通らない."""
        self.set_court_keepout(True)
        try:
            self.run(self.arm, self.free_params, what, self.p['arm_group'])
        finally:
            self.set_court_keepout(False)

    def move_home(self):
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name='home')
        self.run_free('home への移動')

    def set_gripper(self, angle, what):
        self.gripper.set_start_state_to_current_state()
        state = RobotState(self.moveit.get_robot_model())
        state.set_joint_group_positions(self.p['gripper_group'], [angle])
        self.gripper.set_goal_state(robot_state=state)
        self.run(self.gripper, self.gripper_params, what, self.p['gripper_group'])

    def to_planning_frame(self, point: Point, frame: str) -> Point:
        if not frame or frame == self.p['planning_frame']:
            return point
        tf = self.tf_buffer.lookup_transform(self.p['planning_frame'], frame, Time())
        return do_transform_point(PointStamped(point=point), tf).point

    # ---- アクション ----

    def execute(self, goal_handle):
        goal = goal_handle.request
        push = goal.mode == MoveBall.Goal.MODE_PUSH
        ball = goal.ball.position
        target = self.to_planning_frame(goal.target, goal.target_frame)
        self.get_logger().info(
            f'要求: ボール({ball.x:.3f}, {ball.y:.3f}) → 目標({target.x:.3f}, {target.y:.3f}) '
            f'[{"push" if push else "place"}]')

        def stage(name):
            if goal_handle.is_cancel_requested:
                raise MotionError('キャンセルされました')
            feedback = MoveBall.Feedback()
            feedback.stage = name
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'  {name}')

        p = self.p
        obstacles = list(goal.obstacles)
        # 押し出しのときは、目標の push_distance 手前 (ロボット側) にいったん置く
        push_dir = np.array([target.x, target.y])
        push_dir = push_dir / max(np.linalg.norm(push_dir), 1e-6)
        place_xy = np.array([target.x, target.y]) - (push_dir * p['push_distance'] if push else 0)
        place_z = ball.z   # ボールの中心の高さは置いても変わらない

        # 他のボールと、これから動かすボールを障害物として MoveIt に教える
        diameter = goal.ball.diameter or self.court['ball']['diameter']
        names = [f'boccia_obstacle_{i}' for i in range(len(obstacles))]
        for name, other in zip(names, obstacles):
            self.set_ball(name, other.position, other.diameter or diameter)
        moving = 'boccia_moving_ball'
        names.append(moving)
        self.set_ball(moving, ball, diameter)

        result = MoveBall.Result()
        try:
            # 動き出す前に、つかむ・離す・押すときの手が何にも当たらない手首の向きを決めておく.
            # (途中で失敗すると、ボールを持ったまま止まってしまうため)
            # 向きの第一候補は、指が開く向きを近くのボールと直角にしたもの.
            # 押すときは押す向きと直角 (閉じた指の面で押す) で、向きは変えない
            stage('check')
            self.set_ball(moving, None, 0.0, add=False)
            # 指は「開いた状態」と「ボールを握った状態」で届く場所が違うので、両方確かめる
            fingers = (p['gripper_open'], p['gripper_holding'])
            pick_yaw = self.find_free_yaw(
                [((ball.x, ball.y), ball.z + p['grasp_height'], f) for f in fingers],
                choose_yaw_deg((ball.x, ball.y), obstacles), 'つかむ場所')
            place_yaw = self.find_free_yaw(
                [(place_xy, place_z + p['grasp_height'], f) for f in fingers],
                choose_yaw_deg(place_xy, obstacles), '置く場所')
            push_yaw = normalize_yaw_deg(np.degrees(np.arctan2(push_dir[1], push_dir[0])) + 90.0)
            if push:
                offset = p['push_contact_offset']
                behind = place_xy - push_dir * (offset + p['push_standoff'])
                front = np.array([target.x, target.y]) - push_dir * offset
                z = place_z + p['push_height']
                for xy in (behind, (behind + front) / 2, front):
                    if self.hand_collides(xy, z, push_yaw, 0.0):
                        raise MotionError('押し出す道筋で手が他のボールなどに当たるので中止しました')
            self.set_ball(moving, ball, diameter)

            stage('approach')
            self.yaw = pick_yaw
            self.set_gripper(p['gripper_open'], 'グリッパーを開く')
            self.move_hand(ball.x, ball.y, ball.z + p['approach_height'], False, 'ボールの上への移動')
            stage('grasp')
            self.set_ball(moving, None, 0.0, add=False)   # 指でつかむので、障害物から外す
            self.move_hand(ball.x, ball.y, ball.z + p['grasp_height'], True, 'つかむ高さへの下降')
            self.set_gripper(p['gripper_close'], 'グリッパーを閉じる')
            self.attach_ball(moving, ball, diameter)       # ここからはボールを手に付いた物として扱う
            stage('lift')
            self.move_hand(ball.x, ball.y, ball.z + p['lift_height'] + p['grasp_height'], True,
                           '持ち上げ')
            stage('transport')
            self.yaw = place_yaw
            self.move_hand(*place_xy, place_z + p['lift_height'] + p['grasp_height'], False,
                           '目標の上への移動')
            stage('release')
            self.move_hand(*place_xy, place_z + p['release_clearance'] + p['grasp_height'], True,
                           '置く高さへの下降')
            self.attach_ball(moving, None, 0.0, attach=False)
            # 速く開くと指がボールを弾くので、グリッパーだけゆっくり動かす (gripper_params)
            self.set_gripper(p['gripper_open'], 'グリッパーを開く')
            stage('retreat')
            self.move_hand(*place_xy, place_z + p['approach_height'], True, '真上への退避')

            if push:
                stage('push')
                # 押している間、ボールの中心は手 (gripper_base_link) より push_contact_offset だけ前にある.
                # 手を「目標 - その分」まで動かせば、ボールの中心が目標に来る
                self.yaw = push_yaw
                # 指を閉じると指先が下がるので、閉じるのは「立ち入り禁止の箱」を出入りしない短い移動の間だけ
                carry_z = place_z + p['lift_height'] + p['grasp_height']
                self.move_hand(*behind, carry_z, False, 'ボールの後ろの上への移動')
                self.set_gripper(0.0, 'グリッパーを閉じる (押し出し用)')
                self.move_hand(*behind, z, True, 'ボールの後ろへの下降')
                self.move_hand(*front, z, True, '押し出し', self.push_params)
                self.move_hand(*front, carry_z, True, '押し出し後の退避')

            if p['return_home']:
                stage('home')
                self.move_home()
        except Exception as e:   # MotionError 以外 (想定外の例外) でも、必ずアクションを終わらせる
            self.clear_balls(names)
            if not isinstance(e, MotionError):
                self.get_logger().error(f'想定外のエラー: {e!r}')
                e = MotionError(f'想定外のエラー: {e}')
            self.get_logger().error(str(e))
            result.success = False
            result.message = str(e)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result

        self.clear_balls(names)
        result.success = True
        result.message = f'完了 ({"push" if push else "place"}, 目標との誤差は未計測)'
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    node = MoveBallServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
