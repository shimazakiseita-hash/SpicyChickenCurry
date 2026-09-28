# MuJoCo（学習用シミュレーション）

MuJoCo で CRANE-X7 の動作を学習し、学習した方策を Gazebo（ROS 2）で動かす、という流れで使う予定です。

```
mujoco/
├── models/crane_x7/
│   ├── crane_x7.xml   # ロボット本体（urdf_to_mjcf.py で自動生成。手で編集しない）
│   └── scene.xml      # 床・照明・目標表示つきのシーン（これを読み込む）
├── envs/
│   └── crane_x7_reach.py  # 到達タスクの Gymnasium 環境
├── train_reach.py     # 到達タスクを PPO で学習
├── export_policy.py   # 学習結果を ROS ノード用の policy.npz に書き出す
├── eval_reach.py      # policy.npz を MuJoCo で評価・表示
├── policies/          # チームで共有する学習済み方策
│   └── reach_v1.npz
├── runs/              # 学習結果の出力先（git 管理外）
└── tools/
    └── urdf_to_mjcf.py  # crane_x7_description の xacro → MJCF 変換
```

## 到達タスク（学習 → Gazebo で見せる）

手先（指先の間）を、ロボット前方のランダムな目標位置に 5 秒以内で近づけるタスクです。

| 項目 | 内容 |
|---|---|
| 行動 | アーム 7 関節の目標角度の変化量（1 ステップ最大 0.05 rad、20 Hz） |
| 観測 | 関節角・関節速度・手先位置・目標位置・手先→目標（すべて `base_link` 座標系） |
| 報酬 | −（手先と目標の距離）− 小さな行動ペナルティ − 自分や床への接触ペナルティ |
| 目標の範囲 | x 0.15〜0.40 m、y −0.25〜0.25 m、z 0.05〜0.45 m（`base_link` 基準） |

`reach_v1.npz` の成績（300 万ステップ、約 10 分）:
- MuJoCo: 成功率（誤差 20 mm 未満）100%、誤差の中央値 0.9 mm
- Gazebo: 10 回中 10 回が 20 mm 未満（1〜11 mm）

### 学習する

```bash
.venv/bin/python mujoco/train_reach.py                      # 300 万ステップなら --steps 3000000
.venv/bin/python mujoco/eval_reach.py mujoco/runs/reach_<日時>/policy.npz            # 成功率を表示
.venv/bin/python mujoco/eval_reach.py mujoco/runs/reach_<日時>/policy.npz --viewer   # 動きを見る
```

学習の経過は `mujoco/runs/reach_<日時>/progress.csv` の `reach/success_rate` で見られます。
良い結果が出てチームで共有したいときは、`policy.npz` を `mujoco/policies/` にコピーしてコミットしてください。

### Gazebo で動かす

```bash
# ターミナル1
ros2 launch crane_x7_gazebo crane_x7_with_table.launch.py
# ターミナル2
ros2 launch crane_x7_rl reach_demo.launch.py policy_path:=$PWD/mujoco/policies/reach_v1.npz
```

直立姿勢に戻る → 目標（黄色い球）が現れる → 5 秒かけて手先を目標へ、を繰り返します。ログに毎回の到達誤差が出ます。
ノード（`ros2_ws/src/crane_x7_rl`）は numpy だけで推論するので、`.venv` なしで動きます。

観測・行動の定義を変えるときは、`envs/crane_x7_reach.py` と `crane_x7_rl/policy.py` の `build_observation` を両方直してください。
関節名・制御周期・手先オフセットなどの設定値は `policy.npz` に一緒に保存され、ノードはそれを読みます。

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
