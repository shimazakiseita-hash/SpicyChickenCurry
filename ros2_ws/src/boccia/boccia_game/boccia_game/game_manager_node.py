"""試合の進行役. どのボールをどこへ動かすかを決めて、MoveBall アクションに依頼する.

段階 1 の動き:
  サービス /boccia/play_once (std_srvs/Trigger) が呼ばれるたびに、
  自分のチームのボールを 1 個選び (selection.choose_ball)、
  パラメータ target_x, target_y (base_link 座標) の位置へ置く (mode: place) か押し出す (mode: push).
  アームの動作には 15 秒ほどかかるので、サービスは依頼を出したところで返事をし、
  結果はログに出す. 動作中に呼ばれたら断る.

  サービス /boccia/reset (std_srvs/Trigger) で「どのボールを動かしたか」の記録を消す.

  target_x, target_y, mode は動かしている途中でも変えられる:
    ros2 param set /game_manager target_x 0.30

段階 2 以降は、目標位置をジャックボールの位置から決めたり、
投球パラメータ (力加減・角度) を学習済みの方策から決めたりする予定.
"""

import rclpy
import yaml
from boccia_interfaces.action import MoveBall
from boccia_interfaces.msg import Ball, BallArray
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

import math

from boccia_game.selection import choose_ball, find_obstacle

MODES = {'place': MoveBall.Goal.MODE_PLACE, 'push': MoveBall.Goal.MODE_PUSH}


class GameManagerNode(Node):

    def __init__(self):
        super().__init__('game_manager')
        # boccia_bringup/config/court.yaml のパス (launch ファイルから渡す)
        self.declare_parameter('court_config', '')
        self.declare_parameter('target_x', 0.35)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('mode', 'place')   # place / push
        # これより目標に近いボールは「もう置いてある」とみなして選ばない [m]
        self.declare_parameter('done_radius', 0.03)
        # 前に拾った位置からこれ以内のボールは「同じボール」とみなして選ばない [m]
        self.declare_parameter('same_ball_radius', 0.02)
        # 置く場所と他のボールの中心がこれより近いと断る [m]
        # ボールの直径 (60 mm) + 手の中でのボールの位置のずれ (Gazebo で ±1.5 cm ほど) の余裕.
        # (直径 43 mm のときに、中心どうし 5 cm で運んでいるボールがジャックに当たったことがある)
        # 手 (指) が当たるかどうかは move_ball_server が MoveIt で確かめる
        self.declare_parameter('place_clearance', 0.075)
        # 押し出しの道筋と他のボールの中心がこれより近いと、押している手で弾いてしまうので断る [m]
        self.declare_parameter('push_clearance', 0.08)
        # 押し出しのときに手が動く範囲 (move_ball.yaml の push_distance + push_standoff + push_contact_offset)
        self.declare_parameter('push_reach_behind', 0.12)

        court_config = self.get_parameter('court_config').value
        if not court_config:
            raise RuntimeError('court_config パラメータに court.yaml のパスを指定してください')
        with open(court_config) as f:
            self.court = yaml.safe_load(f)
        self.team_type = Ball.TYPE_RED if self.court['team_color'] == 'red' else Ball.TYPE_BLUE

        self.latest_balls = None
        self.picked_from = []   # これまでにボールを拾い上げた位置 (x, y)
        self.busy = False
        self.create_subscription(BallArray, '/boccia/balls', self.on_balls, 10)
        self.move_client = ActionClient(self, MoveBall, '/boccia/move_ball')
        self.create_service(Trigger, '/boccia/play_once', self.on_play_once)
        self.create_service(Trigger, '/boccia/reset', self.on_reset)
        self.get_logger().info(f'試合進行ノードを起動しました (自分の色: {self.court["team_color"]})')

    def on_balls(self, msg: BallArray):
        self.latest_balls = msg

    def on_reset(self, request, response):
        self.picked_from.clear()
        response.success = True
        response.message = '動かしたボールの記録を消しました'
        return response

    def on_play_once(self, request, response):
        response.success = False
        if self.busy:
            response.message = 'アームが動作中です。終わってから呼んでください'
            return response
        if self.latest_balls is None:
            response.message = 'ボールの検出結果をまだ受け取っていません'
            return response
        mode = self.get_parameter('mode').value
        if mode not in MODES:
            response.message = f'mode "{mode}" が不明です (place / push)'
            return response
        if not self.move_client.server_is_ready():
            response.message = 'MoveBall アクションサーバ (/boccia/move_ball) が起動していません'
            return response

        target = (self.get_parameter('target_x').value, self.get_parameter('target_y').value)
        ball = choose_ball(
            list(self.latest_balls.balls), self.team_type, target, self.picked_from,
            self.get_parameter('done_radius').value, self.get_parameter('same_ball_radius').value)
        if ball is None:
            response.message = '動かせる自分のボールがありません (/boccia/reset で記録を消せます)'
            return response

        clearance = self.get_parameter(f'{mode}_clearance').value
        obstacle = find_obstacle(list(self.latest_balls.balls), ball,
                                 self.hand_path(target, mode), clearance)
        if obstacle is not None:
            response.message = (f'目標の近くに他のボール (id {obstacle.id}, '
                                f'{obstacle.position.x:.3f}, {obstacle.position.y:.3f}) があり、'
                                f'{clearance * 100:.0f} cm 以内なので中止しました。目標を変えてください')
            return response

        goal = MoveBall.Goal()
        goal.ball = ball
        goal.target.x, goal.target.y = target
        goal.target_frame = self.latest_balls.header.frame_id
        goal.mode = MODES[mode]
        # 他のボールも渡す (アームが障害物として避け、指が当たらない手首の向きを選ぶ)
        goal.obstacles = [b for b in self.latest_balls.balls if b is not ball]
        self.busy = True
        self.picked_from.append((ball.position.x, ball.position.y))
        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

        response.success = True
        response.message = (f'ボール id {ball.id} ({ball.position.x:.3f}, {ball.position.y:.3f}) を '
                            f'({target[0]:.3f}, {target[1]:.3f}) へ {mode} するよう依頼しました')
        self.get_logger().info(response.message)
        return response

    def hand_path(self, target, mode):
        """置く・押し出すときに手が下りる道筋 (base_link 座標の折れ線)."""
        if mode != 'push':
            return [target]
        # 押し出しは「ロボットの根元 → 目標」の向きに、目標の手前から目標まで手を動かす
        norm = max(math.hypot(*target), 1e-6)
        back = self.get_parameter('push_reach_behind').value
        start = (target[0] - target[0] / norm * back, target[1] - target[1] / norm * back)
        return [start, target]

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('MoveBall の依頼が断られました')
            self.busy = False
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'動作が終わりました: {result.message}')
        else:
            self.get_logger().error(f'動作に失敗しました: {result.message}')
        self.busy = False


def main():
    rclpy.init()
    node = GameManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
