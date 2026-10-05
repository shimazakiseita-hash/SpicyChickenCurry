"""ボールをつかんで目標位置へ置く・押し出すアクションサーバ (MoveIt / moveit_py を使う).

アクション: /boccia/move_ball (boccia_interfaces/action/MoveBall)

動作の流れ (MODE_PLACE):
  approach  : グリッパーを開き、ボールの真上 (approach_height) へ移動
  grasp     : 真下へ下ろして (grasp_height) グリッパーを閉じる
  lift      : 持ち上げる (lift_height)
  transport : 目標位置の真上へ移動
  release   : 下ろしてグリッパーを開く
  retreat   : 真上へ逃げる

MODE_PUSH は release のあと、目標の手前 (push_standoff) から目標方向へ push_distance だけ
手先を水平に動かして押し出す.

起動には MoveIt の設定 (robot_description など) が必要なので、
boccia_bringup/launch/manipulation.launch.py から起動すること.
"""

import rclpy
from boccia_interfaces.action import MoveBall
from rclpy.action import ActionServer
from rclpy.node import Node

STAGES = ['approach', 'grasp', 'lift', 'transport', 'release', 'retreat']


class MoveBallServer(Node):

    def __init__(self):
        super().__init__('move_ball_server')
        for name, default in [
            ('arm_group', 'arm'), ('gripper_group', 'gripper'),
            ('ee_link', 'crane_x7_gripper_base_link'), ('planning_frame', 'base_link'),
            ('approach_height', 0.10), ('grasp_height', 0.07), ('lift_height', 0.08),
            ('gripper_open', 0.9), ('gripper_close', 0.15),
            ('push_standoff', 0.06), ('push_distance', 0.05),
            ('velocity_scaling', 0.3), ('acceleration_scaling', 0.3),
        ]:
            self.declare_parameter(name, default)

        # TODO(段階1): MoveItPy を作り、arm / gripper の planning component を取得する.
        #   crane_x7_examples_py/crane_x7_examples_py/pick_and_place.py が参考になる.
        #   from moveit.planning import MoveItPy
        #   self.moveit = MoveItPy(node_name='move_ball_moveit')
        #   self.arm = self.moveit.get_planning_component(arm_group)
        self.moveit = None

        self.server = ActionServer(self, MoveBall, '/boccia/move_ball', self.execute)
        self.get_logger().info('MoveBall アクションサーバを起動しました (動作は未実装)')

    def execute(self, goal_handle):
        goal = goal_handle.request
        mode = 'push' if goal.mode == MoveBall.Goal.MODE_PUSH else 'place'
        self.get_logger().info(
            f'要求: ボール({goal.ball.position.x:.3f}, {goal.ball.position.y:.3f}) → '
            f'目標({goal.target.x:.3f}, {goal.target.y:.3f}) [{mode}]')

        # TODO(段階1): STAGES の順に手先の目標姿勢を作って計画・実行する.
        #   - 手先は真下向き (グリッパーが下を向く姿勢)
        #   - 各段階の前に feedback.stage を送る
        #   - 計画に失敗したら goal_handle.abort() して理由を message に入れる
        result = MoveBall.Result()
        result.success = False
        result.message = 'まだ実装されていません'
        goal_handle.abort()
        return result


def main():
    rclpy.init()
    node = MoveBallServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
