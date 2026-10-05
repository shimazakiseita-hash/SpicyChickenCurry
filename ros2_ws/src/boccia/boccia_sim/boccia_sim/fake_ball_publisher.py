"""【L0】シナリオファイルのボール位置を、そのまま検出結果として出すノード.

カメラも検出ノードも使わずに、boccia_game と boccia_manipulation の動作確認ができる.

出力: /boccia/balls (BallArray)
パラメータ:
  scenario: シナリオファイル (config/scenario_default.yaml の形式) のパス
  rate:     出力する周期 [Hz]
"""

import rclpy
import yaml
from boccia_interfaces.msg import BallArray
from rclpy.node import Node


class FakeBallPublisher(Node):

    def __init__(self):
        super().__init__('fake_ball_publisher')
        self.declare_parameter('scenario', '')
        self.declare_parameter('rate', 5.0)

        scenario = self.get_parameter('scenario').value
        if not scenario:
            raise RuntimeError('scenario パラメータにシナリオファイルのパスを指定してください')
        with open(scenario) as f:
            self.scenario = yaml.safe_load(f)

        self.pub = self.create_publisher(BallArray, '/boccia/balls', 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.on_timer)
        self.get_logger().info(f'シナリオを読み込みました: {scenario}')

    def on_timer(self):
        msg = BallArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.scenario['frame_id']
        # TODO(段階1): self.scenario['balls'] の各要素から Ball を作って msg.balls に入れる
        #   type の文字列 (jack / red / blue) → Ball.TYPE_JACK / TYPE_RED / TYPE_BLUE
        #   id は 1 から順番、confidence は 1.0、diameter は court.yaml の値
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
