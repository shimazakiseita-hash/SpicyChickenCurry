"""selection.py のテスト (ROS を起動せずに動く)."""

from boccia_interfaces.msg import Ball
from boccia_game.selection import choose_ball

TARGET = (0.35, 0.0)


def make_ball(type_, x, y, id_=0):
    ball = Ball()
    ball.type, ball.id = type_, id_
    ball.position.x, ball.position.y = x, y
    return ball


def choose(balls, picked_from=()):
    return choose_ball(balls, Ball.TYPE_RED, TARGET, list(picked_from),
                       done_radius=0.03, same_ball_radius=0.02)


def test_closest_to_robot_is_chosen():
    near = make_ball(Ball.TYPE_RED, 0.20, 0.05, 1)
    far = make_ball(Ball.TYPE_RED, 0.30, -0.15, 2)
    assert choose([far, near]) is near


def test_other_colors_and_jack_are_ignored():
    jack = make_ball(Ball.TYPE_JACK, 0.15, 0.0)
    blue = make_ball(Ball.TYPE_BLUE, 0.16, 0.0)
    red = make_ball(Ball.TYPE_RED, 0.30, 0.10)
    assert choose([jack, blue, red]) is red


def test_ball_already_at_target_is_skipped():
    done = make_ball(Ball.TYPE_RED, 0.36, 0.01)    # 目標から約 1.4 cm
    red = make_ball(Ball.TYPE_RED, 0.30, 0.10)
    assert choose([done, red]) is red


def test_previously_picked_position_is_skipped():
    # L0 のダミーデータのように、動かしたあとも同じ位置に見え続ける場合
    first = make_ball(Ball.TYPE_RED, 0.20, 0.05)
    second = make_ball(Ball.TYPE_RED, 0.30, -0.15)
    assert choose([first, second], picked_from=[(0.201, 0.049)]) is second


def test_none_when_no_candidate():
    assert choose([]) is None
    assert choose([make_ball(Ball.TYPE_BLUE, 0.2, 0.0)]) is None


def test_find_obstacle_near_target():
    from boccia_game.selection import find_obstacle
    moving = make_ball(Ball.TYPE_RED, 0.2, 0.0)
    other = make_ball(Ball.TYPE_RED, 0.35, -0.06)
    assert find_obstacle([moving, other], moving, [(0.35, -0.05)], 0.07) is other
    assert find_obstacle([moving, other], moving, [(0.35, 0.05)], 0.07) is None


def test_find_obstacle_along_push_path():
    from boccia_game.selection import find_obstacle
    moving = make_ball(Ball.TYPE_RED, 0.27, 0.13)
    other = make_ball(Ball.TYPE_RED, 0.35, -0.06)   # 押し出しの道筋から約 4 cm 横
    path = [(0.30, -0.017), (0.42, -0.02)]
    assert find_obstacle([moving, other], moving, path, 0.07) is other
    assert find_obstacle([moving, other], moving, path, 0.03) is None
