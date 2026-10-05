"""fake_ball_publisher のシナリオ読み込みのテスト (ROS を起動せずに動く).

    colcon test --packages-select boccia_sim && colcon test-result --verbose
"""

from pathlib import Path

import pytest
import yaml
from boccia_interfaces.msg import Ball
from boccia_sim.fake_ball_publisher import scenario_to_balls

SCENARIO = Path(__file__).resolve().parents[1] / 'config' / 'scenario_default.yaml'


def test_default_scenario():
    scenario = yaml.safe_load(SCENARIO.read_text())
    balls = scenario_to_balls(scenario, ball_diameter=0.043, jack_diameter=0.04)

    assert [b.type for b in balls] == [Ball.TYPE_JACK, Ball.TYPE_RED, Ball.TYPE_RED, Ball.TYPE_RED]
    assert [b.id for b in balls] == [1, 2, 3, 4]
    assert balls[0].diameter == pytest.approx(0.04)
    assert balls[1].diameter == pytest.approx(0.043)
    assert balls[0].position.x == pytest.approx(0.40)
    assert all(b.confidence == 1.0 for b in balls)


def test_unknown_type_is_rejected():
    scenario = {'balls': [{'type': 'green', 'x': 0.0, 'y': 0.0, 'z': 0.0}]}
    with pytest.raises(ValueError, match='green'):
        scenario_to_balls(scenario, 0.043, 0.043)
