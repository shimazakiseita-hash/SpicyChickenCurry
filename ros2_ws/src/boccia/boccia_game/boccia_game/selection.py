"""次に動かすボールを選ぶ (ROS ノードから切り離して、テストしやすくしている)."""

import math

from boccia_interfaces.msg import Ball


def distance_xy(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def choose_ball(balls: list[Ball], team_type: int, target: tuple[float, float],
                picked_from: list[tuple[float, float]], done_radius: float,
                same_ball_radius: float) -> Ball | None:
    """自分のチームのボールから、次に動かすものを 1 個選ぶ (無ければ None).

    除外するもの:
      - すでに目標の近く (done_radius 以内) にあるボール
      - 前に拾い上げた位置 (picked_from) の近く (same_ball_radius 以内) にあるボール.
        検出結果が更新されない場合 (L0 のダミーデータなど) に、同じボールを何度も選ばないため
    残りの中から、ロボットの根元 (base_link の原点) に一番近いものを選ぶ (腕を伸ばす距離が短く確実).
    """
    candidates = []
    for ball in balls:
        if ball.type != team_type:
            continue
        x, y = ball.position.x, ball.position.y
        if distance_xy(x, y, *target) <= done_radius:
            continue
        if any(distance_xy(x, y, px, py) <= same_ball_radius for px, py in picked_from):
            continue
        candidates.append(ball)
    if not candidates:
        return None
    return min(candidates, key=lambda b: math.hypot(b.position.x, b.position.y))


def point_segment_distance(p: tuple[float, float], a: tuple[float, float],
                           b: tuple[float, float]) -> float:
    """点 p と線分 ab の距離."""
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / length2))
    return distance_xy(p[0], p[1], ax + t * dx, ay + t * dy)


def find_obstacle(balls: list[Ball], moving: Ball, path: list[tuple[float, float]],
                  clearance: float) -> Ball | None:
    """手やボールが通る道筋 path (折れ線) に、clearance より近い他のボールがあれば返す.

    アーム (MoveIt) は他のボールの位置を知らないので、置く場所や押し出す道筋が他のボールに
    近いと、手でそのボールを弾いてしまう. その前に試合進行の側で止めるために使う.
    """
    for ball in balls:
        if ball is moving:
            continue
        p = (ball.position.x, ball.position.y)
        segments = list(zip(path, path[1:])) or [(path[0], path[0])]
        if any(point_segment_distance(p, a, b) < clearance for a, b in segments):
            return ball
    return None
