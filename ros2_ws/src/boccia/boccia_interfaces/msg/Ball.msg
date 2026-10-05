# 検出されたボール 1 個
#
# 種類は「色」で表す。どちらの色が自分のチームかは boccia_game 側で決める
# (court.yaml の team_color)。段階 2 以降で相手チームを入れるときもこのまま使える。

uint8 TYPE_UNKNOWN = 0
uint8 TYPE_JACK = 1   # ジャックボール (白)
uint8 TYPE_RED = 2    # 赤チームのボール
uint8 TYPE_BLUE = 3   # 青チームのボール

uint8 type

# 追跡用の番号。フレームをまたいで同じボールなら同じ番号にする (追跡しない場合は 0)
uint32 id

# ボールの中心位置 [m]。座標系は BallArray の header.frame_id (通常は base_link)
geometry_msgs/Point position

# 推定した直径 [m] (分からなければ 0)
float32 diameter

# 検出の確からしさ 0.0〜1.0
float32 confidence
