# SpicyChickenCurry

CRANE-X7 を ROS 2 Lyrical で動かすためのワークスペースです。

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
└── rt_manipulators_cpp_ros2/  # rt-net: 実機制御ライブラリ
```

チームで作るパッケージは `ros2_ws/src/` の下に追加してください。
