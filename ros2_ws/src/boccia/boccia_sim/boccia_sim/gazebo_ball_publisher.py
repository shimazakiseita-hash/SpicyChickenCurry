"""【L2】Gazebo の中のボールの本当の位置を、検出結果として出すノード.

カメラの代わりに Gazebo から直接ボールの位置を読むので、アームで動かしたあとの位置も
そのまま反映される (scorer の得点が実際の結果になる). カメラと検出ノードを Gazebo で
動かせるようになったら、それと比べる正解データとしても使える.

入力: Gazebo のトピック /world/<world>/pose/info (gz-transport で直接読む)
出力: /boccia/balls (BallArray, base_link 座標)

対象は gazebo_ball_spawner が置いたモデル (名前が boccia_ball_<番号>_<種類>).
台から落ちたボール (base_link で z < -0.1) は出さない.
"""

import re
import threading

import rclpy
import yaml
from boccia_interfaces.msg import Ball, BallArray
from gz.msgs.pose_v_pb2 import Pose_V
from gz.transport import Node as GzNode
from rclpy.node import Node

from boccia_sim.fake_ball_publisher import TYPE_BY_NAME

NAME_PATTERN = re.compile(r'boccia_ball_(\d+)_(jack|red|blue)$')


class GazeboBallPublisher(Node):

    def __init__(self):
        super().__init__('gazebo_ball_publisher')
        self.declare_parameter('world', 'default')
        self.declare_parameter('base_z_in_gazebo', 1.015)
        self.declare_parameter('court_config', '')
        self.declare_parameter('rate', 5.0)

        court_path = self.get_parameter('court_config').value
        if not court_path:
            raise RuntimeError('court_config パラメータに court.yaml のパスを指定してください')
        with open(court_path) as f:
            court = yaml.safe_load(f)
        self.diameter = {'jack': court['ball']['jack_diameter'], 'red': court['ball']['diameter'],
                         'blue': court['ball']['diameter']}
        self.base_z = self.get_parameter('base_z_in_gazebo').value

        self.lock = threading.Lock()
        self.poses = {}   # モデル名 -> (x, y, z) (Gazebo のワールド座標)
        self.gz_topic = f'/world/{self.get_parameter("world").value}/pose/info'
        self.gz_node = GzNode()
        if not self.gz_node.subscribe(Pose_V, self.gz_topic, self.on_gz_poses):
            raise RuntimeError(f'Gazebo のトピック {self.gz_topic} を購読できません')

        self.pub = self.create_publisher(BallArray, '/boccia/balls', 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.on_timer)
        self.get_logger().info(f'Gazebo のボール位置を {self.gz_topic} から読みます')

    def on_gz_poses(self, msg: Pose_V):
        # gz-transport のスレッドから呼ばれる
        poses = {p.name: (p.position.x, p.position.y, p.position.z)
                 for p in msg.pose if p.name.startswith('boccia_ball_')}
        with self.lock:
            self.poses = poses

    def on_timer(self):
        with self.lock:
            poses = dict(self.poses)
        msg = BallArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        for name, (x, y, z) in sorted(poses.items()):
            match = NAME_PATTERN.match(name)
            if not match or z - self.base_z < -0.1:
                continue
            ball = Ball()
            ball.id = int(match.group(1))
            ball.type = TYPE_BY_NAME[match.group(2)]
            ball.position.x, ball.position.y, ball.position.z = x, y, z - self.base_z
            ball.diameter = float(self.diameter[match.group(2)])
            ball.confidence = 1.0
            msg.balls.append(ball)
        self.pub.publish(msg)

    def destroy_node(self):
        # 購読を止めてから終了しないと、gz-transport のスレッドが残って終了時に落ちる
        self.gz_node.unsubscribe(self.gz_topic)
        super().destroy_node()


def main():
    rclpy.init()
    node = GazeboBallPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
