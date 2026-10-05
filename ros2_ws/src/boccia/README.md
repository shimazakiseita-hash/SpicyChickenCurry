# ミニボッチャ（boccia パッケージ群）

CRANE-X7 に卓上のミニボッチャをプレイさせるための ROS 2 パッケージ群です。
設計製作論実習３の SpicyChickenCurry チームが作りました。

## 何を作るのか

| 段階 | 内容 | 状況 |
|---|---|---|
| **段階 1** | コート上方の固定カメラでボール（ジャック 1 個 + 自分のボール数個）を見つけ、アームでつかんで指定した位置に置く・押し出す | ひな形まで作成済み（中身は TODO） |
| 段階 2 | 投球の力加減・角度を変えて、転がる距離や方向を制御する。ジャックにどれだけ近づけたかを数値化する | 未着手 |
| 段階 3 | MuJoCo + 強化学習で投球パラメータをシミュレーション上で最適化する | 未着手（`mujoco/` に到達タスクの学習の仕組みあり） |

## 全体の流れ（段階 1）

```
 RealSense D435 (コート上方に固定)            ← 実機が無いときは boccia_sim が代わりに画像を出す
   │ カラー画像 / 深度画像 / カメラ情報
   ▼
 ball_detector (boccia_perception)          ← 色でボールを探し、深度から 3 次元位置を求める
   │ /boccia/balls  (BallArray, base_link 座標)
   ├──────────────► scorer (boccia_game)    → /boccia/score (ジャックとの距離)
   ▼
 game_manager (boccia_game)                 ← どのボールをどこへ動かすか決める
   │ /boccia/move_ball  (MoveBall アクション)
   ▼
 move_ball_server (boccia_manipulation)     ← MoveIt で「つかむ → 運ぶ → 置く/押す」
   │
   ▼
 CRANE-X7 (実機 / 仮想モーター / Gazebo)   ← crane_x7_ros の launch で別に起動する
```

## パッケージ

| パッケージ | 言語 | 役割 | 主なファイル |
|---|---|---|---|
| `boccia_interfaces` | (定義のみ) | メッセージ・アクションの定義 | `msg/Ball.msg`, `msg/BallArray.msg`, `msg/Score.msg`, `action/MoveBall.action` |
| `boccia_perception` | Python | ボール検出 | `ball_detector_node.py`, `detection.py`（ROS に依存しない画像処理）, `config/ball_detector.yaml` |
| `boccia_manipulation` | Python | アーム動作（MoveIt） | `move_ball_server.py`, `config/move_ball.yaml`, `config/moveit_py.yaml` |
| `boccia_game` | Python | 試合の進行と得点計算 | `game_manager_node.py`, `scorer_node.py` |
| `boccia_sim` | Python | 実機なしで試すためのダミーデータ | `fake_ball_publisher.py`（L0）, `synthetic_camera_node.py`（L1）, `config/scenario_default.yaml`, `worlds/`（L2） |
| `boccia_bringup` | Python | 起動ファイルと共通設定 | `launch/*.launch.py`, `config/court.yaml`, `config/camera_extrinsics.yaml` |

担当を分けるときの目安: 認識（perception）、アーム（manipulation）、試合・学習（game と `mujoco/`）、ハード・シミュレーション（sim、コートやボールの製作、カメラの取り付け）。

## トピック・アクション・座標系

| 名前 | 種類 | 型 | 内容 |
|---|---|---|---|
| `/camera/color/image_raw` | トピック | `sensor_msgs/Image` | カラー画像（RealSense または合成） |
| `/camera/aligned_depth_to_color/image_raw` | トピック | `sensor_msgs/Image` | カラーに位置合わせした深度（16bit, mm） |
| `/camera/color/camera_info` | トピック | `sensor_msgs/CameraInfo` | カメラの内部パラメータ |
| `/boccia/balls` | トピック | `boccia_interfaces/BallArray` | 検出したボール（`base_link` 座標） |
| `/boccia/score` | トピック | `boccia_interfaces/Score` | ジャックと各ボールの距離 |
| `/boccia/move_ball` | アクション | `boccia_interfaces/action/MoveBall` | ボールを 1 個つかんで置く・押し出す |
| `/boccia/play_once` | サービス | `std_srvs/Trigger` | 「次の 1 球」を動かす（段階 1） |

座標系（TF）:

- `base_link`: CRANE-X7 の根元。x がロボットの正面、y が左、z が上。**ボールの位置は全部この座標で扱う。**
- `camera_link`: RealSense 本体。`base_link → camera_link` は `config/camera_extrinsics.yaml` の値を `camera_tf.launch.py` が流す。
- `camera_color_optical_frame` など: `camera_link` から先は realsense2_camera が流す。

ボールの種類は色で表します（`Ball.TYPE_JACK` / `TYPE_RED` / `TYPE_BLUE`）。どちらの色が自分のチームかは `court.yaml` の `team_color` で決めます。

## 設定ファイル

| ファイル | 中身 | 使うもの |
|---|---|---|
| `boccia_bringup/config/court.yaml` | コートの位置・大きさ、ボールの直径、自分の色 | game, sim（将来は MuJoCo の学習環境からも読む） |
| `boccia_bringup/config/camera_extrinsics.yaml` | 固定カメラの取り付け位置 | `camera_tf.launch.py` |
| `boccia_perception/config/ball_detector.yaml` | 色の範囲（HSV）、トピック名 | ball_detector |
| `boccia_manipulation/config/move_ball.yaml` | つかむ高さ、グリッパーの開閉角、速度 | move_ball_server |
| `boccia_sim/config/scenario_default.yaml` | ダミーデータのボール配置（正解の位置） | fake_ball_publisher, synthetic_camera |

数値はすべて仮の値です。コートやカメラを実際に作ったら測って書き換えてください。

## 動かし方

ビルド（リポジトリの README のセットアップが済んでいる前提）:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

ロボット側（アームを動かすとき）は、どれか 1 つを別のターミナルで起動しておきます。

```bash
ros2 launch crane_x7_examples demo.launch.py use_mock_components:=true   # 仮想モーター（実機なし）
ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0     # 実機
```

### 実機が無いときのダミーデータ（L0 → L1 → L2 の順に作る）

| レベル | 起動 | 何が偽物か | 何を確かめられるか |
|---|---|---|---|
| **L0** | `ros2 launch boccia_bringup dummy_l0.launch.py` | 検出結果そのもの（シナリオのボール位置をそのまま出す） | 試合進行とアーム動作 |
| **L1** | `ros2 launch boccia_bringup dummy_l1.launch.py` | カメラ画像（シナリオの位置にボールがある画像と深度を合成） | 検出・座標変換も含めた全体。正解が分かるので検出の誤差も測れる |
| L2 | （未作成）Gazebo | 物理シミュレーション（コート・ボール・カメラ） | つかむ・転がるところまで含めた全体 |
| 実機 | `ros2 launch boccia_bringup real.launch.py` | なし | 本番 |

検出だけを試したいときは `with_arm:=false` を付けるとアーム動作のノードを起動しません。

1 球動かす（`game_manager` が実装できたら）:

```bash
ros2 service call /boccia/play_once std_srvs/srv/Trigger
ros2 topic echo /boccia/score
```

RealSense の実機が届いたら、rosbag2 で録画しておくと、ロボットなしで何度でも検出を試せます。

```bash
ros2 bag record /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /camera/color/camera_info /tf_static
```

## 実装状況（TODO）

ひな形の段階では、ノードは起動して決められたトピックやアクションをやり取りしますが、中身の処理はまだありません。
各ファイルの `TODO(段階1)` に、やることと手順を書いてあります。

- [ ] `boccia_sim/fake_ball_publisher.py`: シナリオから `Ball` を作って出す（**最初にやる**。L0 が動くようになる）
- [ ] `boccia_game/scorer_node.py`: ジャックとの距離を計算する
- [ ] `boccia_game/game_manager_node.py`: 自分のボールを選んで MoveBall を依頼する
- [ ] `boccia_manipulation/move_ball_server.py`: MoveIt で「つかむ → 運ぶ → 置く/押す」
- [ ] `boccia_sim/synthetic_camera_node.py`: 合成画像を作る（L1）
- [ ] `boccia_perception/detection.py`, `ball_detector_node.py`: 色と深度でボールを検出する
- [ ] `boccia_sim/worlds/`: Gazebo のワールド（L2）
- [ ] カメラのキャリブレーション（下記）

## カメラの位置合わせ（キャリブレーション）

固定カメラでは、カメラがロボットから見てどこにあるか（`camera_extrinsics.yaml`）を正確に測る必要があります。
予定している方法: コート上のロボットから見た位置が分かる場所（またはアームの手先）に AR マーカーを置き、カメラから見たマーカーの位置と組み合わせてカメラの位置を逆算します。
`crane_x7_examples` の `aruco_detection` がマーカー検出の参考になります。

## 段階 2 以降に向けた設計メモ

- **コート・ボールの寸法は `court.yaml` 1 か所**にまとめ、ROS のノードと MuJoCo の学習環境の両方から読む。シミュレーションと実機で寸法がずれないようにするため。
- **`MoveBall` アクション**は「置く・押す」だけ。投球は、力加減・角度を持つ別のアクション（`ThrowBall` など）を `boccia_interfaces` に足す。
- **`/boccia/score` の距離**は、投球の評価や強化学習の報酬にそのまま使える。
- 強化学習は CPU で行う（Intel 内蔵 GPU なので CUDA は使えない）。学習した方策は `crane_x7_rl` と同じく numpy だけで動く形で書き出せば、ROS 側で `.venv` なしに使える。

## ボールの大きさについて

CRANE-X7 のグリッパーは、MuJoCo モデルでの見積もりで指先が約 100 mm まで開きます。
確実につかむには直径 60〜70 mm 以下のボールがよく、`court.yaml` では仮にゴルフボール程度（43 mm）にしています。
公式のボッチャボール（約 85 mm、275 g）はつかめるかどうか際どく、重さもアームの負担になります。

## ライセンス

このディレクトリのコードは Apache License 2.0 です（各 `package.xml`）。
ロボットのモデル（`crane_x7_description`）は株式会社アールティの非商用使用許諾規約に従います。詳しくはリポジトリの [README.md](../../../README.md#ライセンス) を参照してください。
