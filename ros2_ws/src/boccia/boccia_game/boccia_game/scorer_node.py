"""検出されたボールから、ジャックボールとの距離を計算するノード.

入力: /boccia/balls (BallArray)
出力: /boccia/score (Score)

距離はコート面上の水平距離 (x, y のみ) で測る.
段階 2 以降では、この値を「投球の結果」として記録したり、強化学習の報酬に使ったりする.
"""

import rclpy
from boccia_interfaces.msg import Ball, BallArray, Score
from rclpy.node import Node


class ScorerNode(Node):

    def __init__(self):
        super().__init__('scorer')
        self.score_pub = self.create_publisher(Score, '/boccia/score', 10)
        self.create_subscription(BallArray, '/boccia/balls', self.on_balls, 10)

    def on_balls(self, msg: BallArray):
        score = Score()
        score.header = msg.header
        score.closest_type = Ball.TYPE_UNKNOWN
        # TODO(段階1):
        #   1. type == TYPE_JACK のボールを探す (複数あれば confidence が一番高いもの)
        #   2. 見つかれば jack_found = True、jack_position に入れる
        #   3. ジャック以外の各ボールについて水平距離 hypot(dx, dy) を distances に入れる
        #   4. 一番近いボールの type を closest_type に入れる
        score.jack_found = False
        self.score_pub.publish(score)


def main():
    rclpy.init()
    node = ScorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
