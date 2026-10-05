"""シナリオのボールを Gazebo に置く (置き終わったら終了する).

crane_x7_gazebo の crane_x7_with_table.launch.py を起動した状態で使う.
ワールドに最初からある木のブロック (wood_cube_5cm) はボールの位置と重なるので取り除く.
もう一度実行すると、前に置いたボールを消してから置き直す.

注意: Gazebo の球には転がり抵抗が無い (SDF の velocity_decay も今の Gazebo では効かない).
そのままでは少し押しただけで止まらずに転がり続けるので、代わりに Hydrodynamics プラグイン
(本来は水中ロボット用) で速度に比例する抵抗をかけている (rolling_damping).
本物の転がり抵抗とは性質が違う近似なので、段階 2 で転がる距離を扱うときは、
実物のコートで測った転がり方に合うように値を調整すること.

  ros2 run boccia_sim gazebo_ball_spawner --ros-args \
    -p scenario:=<scenario.yaml> -p court_config:=<court.yaml>

パラメータ:
  scenario, court_config: L0 と同じファイル
  world:            Gazebo のワールド名 (table.sdf は default)
  base_z_in_gazebo: Gazebo のワールド座標で見た base_link の高さ (crane_x7_with_table の -z)
  remove_models:    取り除くモデル名のリスト
  mass, friction:   ボールの質量 [kg] と摩擦係数 (ゴルフボール程度)
  rolling_damping:  転がり抵抗の代わりの抵抗 [N/(m/s)] (上の注意を参照)
"""

import subprocess

import rclpy
import yaml
from rclpy.node import Node

COLORS = {'jack': '0.95 0.95 0.95 1', 'red': '0.85 0.1 0.1 1', 'blue': '0.1 0.2 0.85 1'}

BALL_SDF = """<sdf version="1.9"><model name="{name}"><link name="link">
<inertial><mass>{mass}</mass><inertia><ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz></inertia></inertial>
<collision name="collision"><geometry><sphere><radius>{r}</radius></sphere></geometry>
<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface></collision>
<visual name="visual"><geometry><sphere><radius>{r}</radius></sphere></geometry>
<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
</link>
<plugin filename="gz-sim-hydrodynamics-system" name="gz::sim::systems::Hydrodynamics">
<link_name>link</link_name><xU>{damp}</xU><yV>{damp}</yV><zW>0</zW>
<kP>{rdamp}</kP><mQ>{rdamp}</mQ><nR>{rdamp}</nR>
<disable_coriolis>true</disable_coriolis><disable_added_mass>true</disable_added_mass>
</plugin></model></sdf>"""


class GazeboBallSpawner(Node):

    def __init__(self):
        super().__init__('gazebo_ball_spawner')
        for name, default in [
            ('scenario', ''), ('court_config', ''), ('world', 'default'),
            ('base_z_in_gazebo', 1.015), ('remove_models', ['wood_cube_5cm']),
            ('mass', 0.046), ('friction', 1.0), ('rolling_damping', 0.1),
        ]:
            self.declare_parameter(name, default)

    def gz_service(self, service, reqtype, req):
        cmd = ['gz', 'service', '-s', f'/world/{self.get_parameter("world").value}/{service}',
               '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean', '--timeout', '10000',
               '--req', req]
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        return 'data: true' in out.stdout

    def remove(self, name):
        return self.gz_service('remove', 'gz.msgs.Entity', f'name: "{name}" type: MODEL')

    def run(self):
        scenario_path = self.get_parameter('scenario').value
        court_path = self.get_parameter('court_config').value
        if not scenario_path or not court_path:
            raise RuntimeError('scenario と court_config パラメータにファイルのパスを指定してください')
        with open(scenario_path) as f:
            scenario = yaml.safe_load(f)
        with open(court_path) as f:
            court = yaml.safe_load(f)

        for name in self.get_parameter('remove_models').value:
            if self.remove(name):
                self.get_logger().info(f'{name} を取り除きました')

        base_z = self.get_parameter('base_z_in_gazebo').value
        mass = self.get_parameter('mass').value
        mu = self.get_parameter('friction').value
        damping = self.get_parameter('rolling_damping').value
        for i, ball in enumerate(scenario['balls'], start=1):
            kind = ball['type']
            diameter = court['ball']['jack_diameter' if kind == 'jack' else 'diameter']
            r = diameter / 2
            name = f'boccia_ball_{i}_{kind}'
            self.remove(name)
            sdf = BALL_SDF.format(name=name, mass=mass, i=0.4 * mass * r * r, r=r, mu=mu, damp=-damping,
                                  rdamp=-damping * r * r,
                                  color=COLORS[kind]).replace('\n', '')  # 文字列内に改行は使えない
            # 台にめり込まないよう 1 mm 浮かせて置く
            req = (f"sdf: '{sdf}' name: \"{name}\" pose: {{position: "
                   f"{{x: {ball['x']}, y: {ball['y']}, z: {ball['z'] + base_z + 0.001}}}}}")
            ok = self.gz_service('create', 'gz.msgs.EntityFactory', req)
            self.get_logger().info(f'{name} を ({ball["x"]}, {ball["y"]}) に置きました' if ok
                                   else f'{name} を置けませんでした')


def main():
    rclpy.init()
    node = GazeboBallSpawner()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
