"""得点計算の中身 (ROS ノードから切り離して、テストしやすくしている)."""

import math

from boccia_interfaces.msg import Ball


def horizontal_distance(a: Ball, b: Ball) -> float:
    """コート面上の水平距離 [m] (高さの違いは無視する)."""
    return math.hypot(a.position.x - b.position.x, a.position.y - b.position.y)


def find_jack(balls: list[Ball]) -> Ball | None:
    """ジャックボールを返す. 複数検出されていたら confidence が一番高いもの."""
    jacks = [b for b in balls if b.type == Ball.TYPE_JACK]
    if not jacks:
        return None
    return max(jacks, key=lambda b: b.confidence)


def compute_score(balls: list[Ball]) -> tuple[Ball | None, list[Ball], list[float], int]:
    """(ジャック, ジャック以外のボール, それぞれの距離, 一番近いボールの種類) を返す.

    ジャックが見つからないときは (None, [], [], TYPE_UNKNOWN).
    ジャック以外のボールが無いときの「一番近い種類」も TYPE_UNKNOWN.
    """
    jack = find_jack(balls)
    if jack is None:
        return None, [], [], Ball.TYPE_UNKNOWN

    others = [b for b in balls if b.type != Ball.TYPE_JACK]
    distances = [horizontal_distance(jack, b) for b in others]
    if not others:
        return jack, [], [], Ball.TYPE_UNKNOWN
    closest = others[distances.index(min(distances))]
    return jack, others, distances, closest.type
