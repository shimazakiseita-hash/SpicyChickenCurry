"""【L1】シナリオのボール位置から、RealSense と同じ形式のカメラ画像を作って出すノード.

RealSense の実機が無くても、ball_detector_node を含めたパイプライン全体を動かせる.
ボールの本当の位置が分かっているので、検出の誤差を測ることもできる.

出力 (RealSense を camera_namespace:='' camera_name:=camera で起動したときと同じ名前):
  /camera/color/image_raw                   (bgr8, 640x480)
  /camera/aligned_depth_to_color/image_raw  (16UC1, 単位 mm)
  /camera/color/camera_info
フレーム: camera_color_optical_frame (camera_link からの TF もこのノードが出す)

パラメータ:
  scenario:      シナリオファイルのパス
  court_config:  court.yaml のパス (ボールの直径、コート面の高さ)
  rate:          出力する周期 [Hz]
  width, height, fx, fy, cx, cy: カメラの内部パラメータ (D435 のカラー 640x480 の代表値)
  depth_noise_std: 深度に加えるノイズの標準偏差 [m]
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class SyntheticCameraNode(Node):

    def __init__(self):
        super().__init__('synthetic_camera')
        for name, default in [
            ('scenario', ''), ('court_config', ''), ('rate', 10.0),
            ('width', 640), ('height', 480),
            ('fx', 615.0), ('fy', 615.0), ('cx', 320.0), ('cy', 240.0),
            ('depth_noise_std', 0.002),
        ]:
            self.declare_parameter(name, default)

        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(
            Image, '/camera/aligned_depth_to_color/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.on_timer)
        self.get_logger().info('合成カメラノードを起動しました (画像生成は未実装)')

    def on_timer(self):
        # TODO(段階1):
        #   1. TF で base_link → camera_color_optical_frame を取得
        #      (camera_link → optical frame の TF は RealSense と同じ値をこのノードが static で出す)
        #   2. 背景 (コート面) の画像と深度を作る: 各画素の視線とコート面 (z = surface_z) の交点
        #   3. 各ボールについて、視線と球の交点を計算して色と深度を上書きする
        #      (簡単にするなら、中心を投影して半径 fx * r / 距離 の円を描くだけでもよい)
        #   4. 深度にノイズを足し、mm 単位の uint16 にする
        #   5. 3 つのメッセージに同じ stamp を付けて出す
        pass


def main():
    rclpy.init()
    node = SyntheticCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
