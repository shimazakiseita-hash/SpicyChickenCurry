"""scoring.py のテスト (ROS を起動せずに動く).

    colcon test --packages-select boccia_game && colcon test-result --verbose
"""

import pytest
from boccia_interfaces.msg import Ball
from boccia_game.scoring import compute_score


def make_ball(type_, x, y, z=0.02, confidence=1.0):
    ball = Ball()
    ball.type = type_
    ball.position.x, ball.position.y, ball.position.z = x, y, z
    ball.confidence = confidence
    return ball


def test_distances_and_closest():
    jack = make_ball(Ball.TYPE_JACK, 0.40, 0.05)
    red = make_ball(Ball.TYPE_RED, 0.40, 0.15)     # 0.10 m
    blue = make_ball(Ball.TYPE_BLUE, 0.43, 0.09)   # 0.05 m (3-4-5)
    found, others, distances, closest = compute_score([red, jack, blue])

    assert found is jack
    assert others == [red, blue]
    assert distances == pytest.approx([0.10, 0.05])
    assert closest == Ball.TYPE_BLUE


def test_height_is_ignored():
    jack = make_ball(Ball.TYPE_JACK, 0.0, 0.0, z=0.02)
    lifted = make_ball(Ball.TYPE_RED, 0.03, 0.04, z=0.50)
    _, _, distances, _ = compute_score([jack, lifted])
    assert distances == pytest.approx([0.05])


def test_most_confident_jack_is_used():
    weak = make_ball(Ball.TYPE_JACK, 1.0, 1.0, confidence=0.3)
    strong = make_ball(Ball.TYPE_JACK, 0.0, 0.0, confidence=0.9)
    red = make_ball(Ball.TYPE_RED, 0.1, 0.0)
    found, others, distances, _ = compute_score([weak, strong, red])
    assert found is strong
    assert others == [red]   # 使わなかったジャックはボールの一覧に入れない
    assert distances == pytest.approx([0.1])


def test_no_jack():
    red = make_ball(Ball.TYPE_RED, 0.1, 0.0)
    assert compute_score([red]) == (None, [], [], Ball.TYPE_UNKNOWN)


def test_only_jack():
    jack = make_ball(Ball.TYPE_JACK, 0.0, 0.0)
    assert compute_score([jack]) == (jack, [], [], Ball.TYPE_UNKNOWN)
