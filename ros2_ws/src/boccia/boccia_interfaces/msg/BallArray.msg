# ある時刻に検出されたボールの一覧 (トピック /boccia/balls)
#
# header.frame_id: ボール位置の座標系 (通常は base_link)
# header.stamp:    元になったカメラ画像の時刻

std_msgs/Header header
Ball[] balls
