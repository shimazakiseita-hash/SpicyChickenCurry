"""検出されたボールから、ジャックボールとの距離を計算するノード.

入力: /boccia/balls (BallArray)
出力: /boccia/score (Score)

距離はコート面上の水平距離 (x, y のみ) で測る. 計算の中身は scoring.py.
段階 2 以降では、この値を「投球の結果」として記録したり、強化学習の報酬に使ったりする.
"""

import rclpy
from boccia_interfaces.msg import BallArray, Score
from rclpy.node import Node

from boccia_game.scoring import compute_score


class ScorerNode(Node):

    def __init__(self):
        super().__init__('scorer')
        self.score_pub = self.create_publisher(Score, '/boccia/score', 10)
        self.create_subscription(BallArray, '/boccia/balls', self.on_balls, 10)

    def on_balls(self, msg: BallArray):
        jack, others, distances, closest_type = compute_score(list(msg.balls))

        score = Score()
        score.header = msg.header
        score.jack_found = jack is not None
        if jack is not None:
            score.jack_position = jack.position
        score.balls = others
        score.distances = distances
        score.closest_type = closest_type
        self.score_pub.publish(score)

        if jack is None:
            self.get_logger().warn('ジャックボールが見つかりません', throttle_duration_sec=5.0)


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
