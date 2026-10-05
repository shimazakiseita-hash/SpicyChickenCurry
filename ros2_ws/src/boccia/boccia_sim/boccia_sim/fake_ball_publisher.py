"""【L0】シナリオファイルのボール位置を、そのまま検出結果として出すノード.

カメラも検出ノードも使わずに、boccia_game と boccia_manipulation の動作確認ができる.

出力: /boccia/balls (BallArray)
パラメータ:
  scenario:     シナリオファイル (config/scenario_default.yaml の形式) のパス
  court_config: court.yaml のパス (ボールの直径を読む)
  rate:         出力する周期 [Hz]
"""

import rclpy
import yaml
from boccia_interfaces.msg import Ball, BallArray
from rclpy.node import Node

TYPE_BY_NAME = {'jack': Ball.TYPE_JACK, 'red': Ball.TYPE_RED, 'blue': Ball.TYPE_BLUE}


def scenario_to_balls(scenario: dict, ball_diameter: float, jack_diameter: float) -> list[Ball]:
    """シナリオの balls を Ball のリストにする. id は書かれた順に 1 から振る."""
    balls = []
    for i, entry in enumerate(scenario['balls'], start=1):
        name = entry['type']
        if name not in TYPE_BY_NAME:
            raise ValueError(f'{i} 個目のボールの type "{name}" が不明です (jack / red / blue)')
        ball = Ball()
        ball.type = TYPE_BY_NAME[name]
        ball.id = i
        ball.position.x = float(entry['x'])
        ball.position.y = float(entry['y'])
        ball.position.z = float(entry['z'])
        ball.diameter = float(jack_diameter if name == 'jack' else ball_diameter)
        ball.confidence = 1.0
        balls.append(ball)
    return balls


class FakeBallPublisher(Node):

    def __init__(self):
        super().__init__('fake_ball_publisher')
        self.declare_parameter('scenario', '')
        self.declare_parameter('court_config', '')
        self.declare_parameter('rate', 5.0)

        scenario_path = self.get_parameter('scenario').value
        court_path = self.get_parameter('court_config').value
        if not scenario_path or not court_path:
            raise RuntimeError('scenario と court_config パラメータにファイルのパスを指定してください')
        with open(scenario_path) as f:
            scenario = yaml.safe_load(f)
        with open(court_path) as f:
            court = yaml.safe_load(f)

        self.frame_id = scenario['frame_id']
        self.balls = scenario_to_balls(
            scenario, court['ball']['diameter'], court['ball']['jack_diameter'])

        self.pub = self.create_publisher(BallArray, '/boccia/balls', 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.on_timer)
        self.get_logger().info(
            f'シナリオを読み込みました: {scenario_path} (ボール {len(self.balls)} 個)')

    def on_timer(self):
        msg = BallArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.balls = self.balls
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeBallPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
