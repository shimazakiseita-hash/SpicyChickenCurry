# Gazebo ワールド（L2、未作成）

L0・L1 で一通り動くようになったら、ここに Gazebo 用のワールドファイル（`.sdf`）を置きます。

予定している中身:

- 台とミニボッチャのコート（`boccia_bringup/config/court.yaml` と同じ寸法）
- ジャックボールと赤・青のボール（摩擦・転がり抵抗を設定した球）
- コート上方の固定カメラ（RGB-D センサー。`camera_extrinsics.yaml` と同じ位置）
- カメラの画像は `ros_gz_bridge` で RealSense と同じトピック名（`/camera/color/image_raw` など）に流す

CRANE-X7 本体は `crane_x7_gazebo` の仕組み（`crane_x7_with_table.launch.py`）を参考に、このワールドに出す。
