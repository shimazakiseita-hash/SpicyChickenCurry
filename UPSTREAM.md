# 取り込んでいる外部パッケージ

`ros2_ws/src` 以下の次のパッケージは、株式会社アールティ（rt-net）のリポジトリを `.git` なしでコピーしたものです。
各ディレクトリの `LICENSE` と著作権表示はそのまま残しています（削除・改変しないこと）。

| ディレクトリ | 元リポジトリ | ブランチ | コミット | ライセンス |
|---|---|---|---|---|
| `crane_x7_description` | https://github.com/rt-net/crane_x7_description | jazzy | `61865950f741f0d5a35e28eb0a7d2501cac86e7b` | **アールティ 非商用使用許諾規約** |
| `crane_x7_ros` | https://github.com/rt-net/crane_x7_ros | jazzy | `3761ab179dbcbd7432b28968340e2225978006de` | Apache License 2.0 |
| `rt_manipulators_cpp_ros2` | https://github.com/rt-net/rt_manipulators_cpp | ros2 | `537d5059fcda43ffc25379a36de1a8367244ddd4` | Apache License 2.0 |

`crane_x7_description`（CRANE-X7 の URDF・メッシュ）だけは Apache ではなく、アールティの非商用使用許諾規約です。
このリポジトリでの扱いは [README.md の「ライセンス」](README.md#ライセンス) を参照してください。
`mujoco/models/crane_x7/` の MuJoCo モデルはこの URDF・メッシュから作ったものなので、同じ規約に従います。

## ROS 2 Lyrical 向けの変更

公式は Jazzy 向けで Lyrical ブランチが無いため、`crane_x7_ros` に次の修正を入れています。
差分は [docs/patches/crane_x7_ros-lyrical.patch](docs/patches/crane_x7_ros-lyrical.patch) にあります。

- `ament_target_dependencies` の削除への対応（`target_link_libraries` に置き換え）
- ros2_control の `on_init(const HardwareInfo &)` → `on_init(const HardwareComponentInterfaceParams &)`
- VTK 9.5 で `JsonCpp::JsonCpp` が見つからない問題への対応（`find_package(jsoncpp)` を追加）
- message_filters のヘッダ名変更（`.h` → `.hpp`）と QoS 引数の必須化
- OpenCV 4.7 以降の aruco API 変更（`getPredefinedDictionary` の戻り値）
- Gazebo 用・実機用の launch で、spawner に `--param-file` でコントローラ設定を渡す（Lyrical ではこれが無いとアームとグリッパーのコントローラが起動しない）

## upstream を更新したいとき

1. 元リポジトリを別の場所に clone し、更新したいコミットを checkout する
2. このリポジトリの該当ディレクトリを中身ごと置き換える（`.git` はコピーしない）
3. `crane_x7_ros` の場合は `cd ros2_ws/src/crane_x7_ros && git apply ../../../docs/patches/crane_x7_ros-lyrical.patch` を試し、当たらない部分は手で直す
4. 上の表のコミットを更新する
