"""試合の進行役. どのボールをどこへ動かすかを決めて、MoveBall アクションに依頼する.

段階 1 の動き:
  サービス /boccia/play_once (std_srvs/Trigger) が呼ばれるたびに、
  まだ動かしていない自分のチームのボールを 1 個選び、
  パラメータ target_x, target_y (base_link 座標) の位置へ置く (または押し出す).

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


class GameManagerNode(Node):

    def __init__(self):
        super().__init__('game_manager')
        # boccia_bringup/config/court.yaml のパス (launch ファイルから渡す)
        self.declare_parameter('court_config', '')
        self.declare_parameter('target_x', 0.35)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('mode', 'place')   # place / push

        court_config = self.get_parameter('court_config').value
        if not court_config:
            raise RuntimeError('court_config パラメータに court.yaml のパスを指定してください')
        with open(court_config) as f:
            self.court = yaml.safe_load(f)
        self.team_type = Ball.TYPE_RED if self.court['team_color'] == 'red' else Ball.TYPE_BLUE

        self.latest_balls = None
        self.create_subscription(BallArray, '/boccia/balls', self.on_balls, 10)
        self.move_client = ActionClient(self, MoveBall, '/boccia/move_ball')
        self.create_service(Trigger, '/boccia/play_once', self.on_play_once)
        self.get_logger().info(f'試合進行ノードを起動しました (自分の色: {self.court["team_color"]})')

    def on_balls(self, msg: BallArray):
        self.latest_balls = msg

    def on_play_once(self, request, response):
        if self.latest_balls is None:
            response.success = False
            response.message = 'ボールの検出結果をまだ受け取っていません'
            return response

        # TODO(段階1):
        #   1. latest_balls から type == self.team_type のボールを選ぶ
        #      (すでに目標の近くにあるボールは除く、など)
        #   2. MoveBall.Goal を作る (ball, target=(target_x, target_y), target_frame, mode)
        #   3. self.move_client.send_goal_async(goal) で依頼し、結果をログに出す
        response.success = False
        response.message = 'まだ実装されていません'
        return response


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
