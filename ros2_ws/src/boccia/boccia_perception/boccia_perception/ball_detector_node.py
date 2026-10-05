"""固定カメラのカラー・深度画像からボールを検出し、base_link 座標で出すノード.

入力:  color_topic / depth_topic (カラーに位置合わせ済みの深度) / camera_info_topic
出力:  /boccia/balls (boccia_interfaces/BallArray, header.frame_id = target_frame)

処理の流れ:
  1. 3 つのトピックを時刻で揃えて受け取る (message_filters)
  2. 色でボール候補を探す (detection.find_blobs)
  3. 深度から 3 次元位置を求める (カメラ座標系)
  4. TF でカメラ座標系 → target_frame (base_link) に変換
  5. BallArray として出す
"""

import rclpy
from boccia_interfaces.msg import BallArray
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from boccia_perception import detection


class BallDetectorNode(Node):

    def __init__(self):
        super().__init__('ball_detector')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('target_frame', 'base_link')
        for kind in ('jack', 'red', 'red2', 'blue'):
            self.declare_parameter(f'{kind}_hsv_low', [0, 0, 0])
            self.declare_parameter(f'{kind}_hsv_high', [179, 255, 255])
        self.declare_parameter('min_radius_px', 5.0)
        self.declare_parameter('max_radius_px', 80.0)
        self.declare_parameter('min_depth', 0.2)
        self.declare_parameter('max_depth', 2.0)

        self.target_frame = self.get_parameter('target_frame').value
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.balls_pub = self.create_publisher(BallArray, '/boccia/balls', 10)
        subs = [
            Subscriber(self, Image, self.get_parameter('color_topic').value),
            Subscriber(self, Image, self.get_parameter('depth_topic').value),
            Subscriber(self, CameraInfo, self.get_parameter('camera_info_topic').value),
        ]
        self.sync = ApproximateTimeSynchronizer(subs, queue_size=10, slop=0.05)
        self.sync.registerCallback(self.on_images)
        self.get_logger().info('ボール検出ノードを起動しました (検出処理は未実装)')

    def hsv_ranges(self):
        get = self.get_parameter
        pair = lambda k: (get(f'{k}_hsv_low').value, get(f'{k}_hsv_high').value)  # noqa: E731
        return {'jack': [pair('jack')], 'red': [pair('red'), pair('red2')], 'blue': [pair('blue')]}

    def on_images(self, color_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        self.camera_model.fromCameraInfo(info_msg)
        color = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

        blobs = detection.find_blobs(
            color, self.hsv_ranges(),
            self.get_parameter('min_radius_px').value, self.get_parameter('max_radius_px').value)

        out = BallArray()
        out.header.stamp = color_msg.header.stamp
        out.header.frame_id = self.target_frame
        for blob in blobs:
            # TODO(段階1):
            #   d = detection.depth_at(depth, blob.u, blob.v, blob.radius_px)
            #   ray = self.camera_model.projectPixelTo3dRay(self.camera_model.rectifyPoint((u, v)))
            #   カメラ座標の点 = ray * (d + 半径) / ray[2]   (frame: color_msg.header.frame_id)
            #   self.tf_buffer.transform(PointStamped, self.target_frame) で base_link に変換
            #   Ball (type, position, diameter, confidence) を作って out.balls に追加
            pass
        self.balls_pub.publish(out)


def main():
    rclpy.init()
    node = BallDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
