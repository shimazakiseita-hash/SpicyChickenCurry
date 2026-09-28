# MuJoCo（学習用シミュレーション）

MuJoCo で CRANE-X7 の動作を学習し、学習した方策を Gazebo（ROS 2）で動かす、という流れで使う予定です。

```
mujoco/
├── models/crane_x7/
│   ├── crane_x7.xml   # ロボット本体（urdf_to_mjcf.py で自動生成。手で編集しない）
│   └── scene.xml      # 床・照明つきのシーン（これを読み込む）
└── tools/
    └── urdf_to_mjcf.py  # crane_x7_description の xacro → MJCF 変換
```

## セットアップ

リポジトリのルートで実行します。`python3-venv` が必要です（`sudo apt install python3-venv`）。

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt
```

`--system-site-packages` を付けるのは、同じ Python から ROS の `rclpy` なども使えるようにするためです。
PyTorch は CPU 版です（このチームの PC には NVIDIA GPU が無いため）。

## モデルを見る

```bash
.venv/bin/python -m mujoco.viewer --mjcf=mujoco/models/crane_x7/scene.xml
```

右側の Control パネルのスライダーで各関節を動かせます。
VS Code のターミナルから開いてウィンドウが出ない・落ちる場合は、ルートの README の「トラブルシューティング」を参照してください。

## モデルの中身

- 関節・リンク・メッシュ・関節の可動範囲は URDF と同じ（関節名も同じなので、ROS 側とそのまま対応が取れる）
- アクチュエータは 8 個、すべて位置制御: アーム 7 軸（`kp=100`）+ グリッパー `crane_x7_gripper_finger_a_joint`（`kp=20`）
  - `ctrl` に目標角度 [rad] を入れると、その角度に向かって動く（ROS の `joint_trajectory_controller` と同じ考え方）
  - 力の上限は URDF の `effort`（肩 2 軸は 10 N·m、それ以外 4 N·m）
- `finger_b` は `finger_a` に連動する（URDF の mimic を等式拘束に変換）
- visual メッシュは見た目だけ（group 2）、collision メッシュは当たり判定だけ（group 3、ビューアでは非表示）
- タイムステップ 2ms。実時間の約 370 倍の速さでシミュレーションできる（CPU 1 コア）

## モデルを作り直す

crane_x7_description の URDF を変えたときや、ゲインなどを調整したいときは、`tools/urdf_to_mjcf.py` の定数を変えて再生成します。

```bash
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
.venv/bin/python mujoco/tools/urdf_to_mjcf.py
```
