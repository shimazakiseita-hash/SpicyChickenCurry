# ジャックボールと各ボールの距離 (トピック /boccia/score)
#
# 段階 2 以降の「どれだけ近づけたか」の評価や、強化学習の報酬の元にする。

std_msgs/Header header

# ジャックボールが見つかっているか (false のときは以下の値は無効)
bool jack_found
geometry_msgs/Point jack_position

# ジャック以外のボールと、それぞれのジャックまでの水平距離 [m] (同じ順番)
Ball[] balls
float32[] distances

# ジャックに一番近いボールの種類 (Ball.TYPE_*)
uint8 closest_type
