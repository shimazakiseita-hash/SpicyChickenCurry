"""ボール検出の中身 (ROS に依存しない関数だけを置く).

ROS ノード (ball_detector_node.py) から呼ばれる. ROS を起動しなくても、
保存した画像を読み込んでこの関数だけを試せるようにしておくと調整が楽.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Blob:
    """画像上で見つかったボール候補."""

    kind: str          # 'jack' / 'red' / 'blue'
    u: float           # 中心の画素位置 (横)
    v: float           # 中心の画素位置 (縦)
    radius_px: float   # 画像上の半径
    score: float       # 確からしさ 0〜1


def find_blobs(bgr_image: np.ndarray, hsv_ranges: dict, min_radius_px: float,
               max_radius_px: float) -> list[Blob]:
    """色の範囲でボール候補を探す.

    hsv_ranges: {'jack': [(low, high)], 'red': [(low, high), (low2, high2)], 'blue': [...]}

    TODO(段階1):
      1. cv2.cvtColor で HSV に変換
      2. 種類ごとに cv2.inRange でマスクを作る (赤は 2 つの範囲の OR)
      3. ノイズ除去 (cv2.morphologyEx の OPEN/CLOSE)
      4. cv2.findContours → cv2.minEnclosingCircle で中心と半径
      5. 半径が範囲外のもの、円らしくないもの (面積 / 円の面積 が小さい) を捨てる
    """
    return []


def depth_at(depth_image: np.ndarray, u: float, v: float, radius_px: float,
             depth_scale: float = 0.001) -> float | None:
    """ボール中心付近の深度 [m] を返す (取れなければ None).

    RealSense の深度画像は 16bit 整数 [mm] なので depth_scale=0.001 を掛ける.

    TODO(段階1):
      - 中心 1 画素だけだと穴 (0) やノイズに弱いので、半径の半分くらいの範囲の
        0 以外の値の中央値を使う
      - 深度はボールの「表面」までの距離なので、中心にするには半径分を足す
        (pixel_to_point 側でカメラの向きに沿って足す)
    """
    return None
