"""手首の向き (yaw) の決め方のテスト (ROS を起動せずに動く)."""

import math

import pytest
from boccia_interfaces.msg import Ball
from boccia_manipulation.move_ball_server import (
    DEFAULT_YAW_DEG, choose_yaw_deg, normalize_yaw_deg)


def make_ball(x, y):
    ball = Ball()
    ball.position.x, ball.position.y = x, y
    return ball


def opening_axis(yaw_deg):
    """yaw のとき指が開く向き (base_link の xy)."""
    return (math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg)))


@pytest.mark.parametrize('yaw, expected', [(-90, -90), (90, -90), (0, 0), (180, 0), (10, -170)])
def test_normalize_yaw(yaw, expected):
    assert normalize_yaw_deg(yaw) == pytest.approx(expected)


def test_default_when_no_ball_nearby():
    assert choose_yaw_deg((0.3, 0.0), [make_ball(0.6, 0.3)]) == DEFAULT_YAW_DEG


@pytest.mark.parametrize('dx, dy', [(0.05, 0.0), (0.0, 0.05), (0.04, 0.03), (-0.03, -0.04)])
def test_fingers_open_perpendicular_to_nearest_ball(dx, dy):
    yaw = choose_yaw_deg((0.3, 0.0), [make_ball(0.3 + dx, dy), make_ball(0.3, 0.11)])
    ax, ay = opening_axis(yaw)
    # 指が開く向きと、近いボールへの向きが直角 (内積が 0)
    assert ax * dx + ay * dy == pytest.approx(0.0, abs=1e-9)
    assert -180.0 < yaw <= 0.0
