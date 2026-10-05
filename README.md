# SpicyChickenCurry

設計製作論実習３の 「SpicyChickenCurry 」チームのリポジトリです。

ロボットアーム CRANE-X7 を ROS 2 Lyrical で動かすためのワークスペースです。
MuJoCo で動作を学習し、Gazebo（シミュレーション）や実機で動かします。

## 動作環境

- Ubuntu 26.04
- ROS 2 Lyrical（`ros-lyrical-desktop`）
- Gazebo（`ros-lyrical-ros-gz`）

公式の crane_x7_ros は Ubuntu 24.04 + Jazzy 向けです。Lyrical で動かすために入れた修正は [UPSTREAM.md](UPSTREAM.md) にまとめています。

## セットアップ

ROS 2 Lyrical と Gazebo が入っている前提です。

```bash
git clone https://github.com/shimazakiseita-hash/SpicyChickenCurry.git
cd SpicyChickenCurry/ros2_ws

source /opt/ros/lyrical/setup.bash
sudo rosdep init   # 初めて rosdep を使う場合のみ
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

`CMake Deprecation Warning` と `tests_require` の警告は無視して大丈夫です。

毎回 source するのが面倒なら、`~/.bashrc` に次を追加してください（パスは clone した場所に合わせる）。

```bash
source /opt/ros/lyrical/setup.bash
source ~/SpicyChickenCurry/ros2_ws/install/setup.bash
```

## シミュレーションで動かす

```bash
# ターミナル1: Gazebo + MoveIt + RViz
ros2 launch crane_x7_gazebo crane_x7_with_table.launch.py

# ターミナル2: サンプル
ros2 launch crane_x7_examples example.launch.py example:=pose_groupstate use_sim_time:=true
```

`example:=` には `gripper_control` / `joint_values` / `cartesian_path` / `pick_and_place` なども指定できます。

## MuJoCo（学習用）

MuJoCo で学習し、Gazebo で見せる構成にしています。セットアップと使い方は [mujoco/README.md](mujoco/README.md) を参照してください。

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python -m mujoco.viewer --mjcf=mujoco/models/crane_x7/scene.xml
```

MuJoCo で学習した到達タスクの方策を Gazebo で動かすデモ（Gazebo を起動した状態で）:

```bash
ros2 launch crane_x7_rl reach_demo.launch.py policy_path:=$PWD/mujoco/policies/reach_v1.npz
```

## ミニボッチャ（開発中）

CRANE-X7 にミニボッチャをプレイさせるプロジェクトです。段階 1 では「固定カメラでボールを見つけ、アームでつかんで目標位置に置く・押し出す」までを作ります。
パッケージ構成、ノードの役割、実機が無くても試せるダミーデータの使い方は [ros2_ws/src/boccia/README.md](ros2_ws/src/boccia/README.md) を参照してください。

```bash
# ダミーデータ (L0) で動かす: ロボット側を仮想モーターで起動してから
ros2 launch crane_x7_examples demo.launch.py use_mock_components:=true
ros2 launch boccia_bringup dummy_l0.launch.py
```

## トラブルシューティング

### VS Code のターミナルから起動すると RViz / Gazebo が落ちる

`symbol lookup error: /snap/core20/.../libpthread.so.0` が出る場合、VS Code が snap 版で、その環境変数が引き継がれているのが原因です。
普通の端末から起動するか、VS Code のターミナルで先に次を実行してください。

```bash
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR LOCPATH
export XDG_DATA_DIRS="$XDG_DATA_DIRS_VSCODE_SNAP_ORIG" XDG_CONFIG_DIRS="$XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG"
```

### `/usr/local` に単体版の rt_manipulators_cpp / DynamixelSDK を入れている場合

ワークスペースの `install/setup.bash` を source せずに実行すると、`/usr/local/lib` の古い方が読み込まれることがあります。
`ldd install/crane_x7_control/lib/libcrane_x7_hardware.so | grep -E "rt_manip|dynamixel"` でどちらが使われているか確認できます。

## ディレクトリ構成

```
ros2_ws/src/
├── crane_x7_description/      # rt-net: URDF・メッシュ
├── crane_x7_ros/              # rt-net: 制御・MoveIt・Gazebo・サンプル（Lyrical 向けに修正済み）
├── rt_manipulators_cpp_ros2/  # rt-net: 実機制御ライブラリ
├── crane_x7_rl/               # チーム: MuJoCo で学習した方策を動かすノード
└── boccia/                    # チーム: ミニボッチャ (6 パッケージ)
```

チームで作るパッケージは `ros2_ws/src/` の下に追加してください。rt-net のパッケージ（`crane_x7_*`、`rt_manipulators_cpp_ros2`）は、Lyrical 対応以外では書き換えないでください（理由は [UPSTREAM.md](UPSTREAM.md)）。

## ライセンス

このリポジトリには、ライセンスの異なるものが混ざっています。

| 対象 | ライセンス |
|---|---|
| `ros2_ws/src/crane_x7_description/`（CRANE-X7 の URDF・メッシュ） | 株式会社アールティ **非商用使用許諾規約**（[LICENSE](ros2_ws/src/crane_x7_description/LICENSE)） |
| `mujoco/models/crane_x7/`（上の URDF・メッシュから作った MuJoCo モデル） | 同上（元の規約に従う） |
| `ros2_ws/src/crane_x7_ros/`、`ros2_ws/src/rt_manipulators_cpp_ros2/` | Apache License 2.0（株式会社アールティ） |
| チームで作ったもの（`boccia/`、`crane_x7_rl/`、`mujoco/` のコードなど） | Apache License 2.0（各 `package.xml` に記載） |

### 非商用使用許諾規約に沿っているか

`crane_x7_description` の規約では、次の点が関係します。

- **使ってよい目的**（第 2 条）: 「教育機関において非商業的な学習、教育または研究を目的とする学生および教職員による使用」は非商用使用に当たります。大学の授業（設計製作論実習３）での使用はこれに該当します。
- **第三者にアクセスさせること**（第 5 条（ウ））: アールティの事前の承諾なく第三者にアクセスさせることは禁止されていますが、「非商業目的の研究または教育の一環」であれば例外とされています。このリポジトリを**授業の関係者（チームメンバー、教員、来年度以降の受講生）に Private で共有する**のは、この例外の範囲と考えています。
- **してはいけないこと**: 商用目的での使用（販売・貸与なども含む）、`LICENSE` や著作権表示を消すこと、規約を他人に譲渡・サブライセンスすること。
- 使用許諾はアールティがいつでも取り消せる（第 4 条）ことにも注意してください。

**リポジトリを Public にしたり、授業以外（コンテストや企業との共同作業など）で使ったりする場合**は、`crane_x7_description` を含めたまま公開してよいかを、事前に担当教員またはアールティに確認してください。
（この節はチームが規約を読んで判断したもので、法的な助言ではありません。）
