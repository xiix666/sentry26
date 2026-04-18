# 裁判系统通信消息（电控发的消息）
# 话题：/receiveLLC_pack

# 比赛信息
# 当前比赛状态
int32 game_progress
# 比赛剩余时间
int32 stage_remain_time
# 红蓝方（红方为1,蓝方为0）
int32 red_blue

# 己方信息
# 己方机器人血量
int32 self_hero_hp
int32 self_infantry_hp
int32 self_sentry_hp

# 哨兵位置
float32 self_pose_x
float32 self_pose_y
# 允许发弹量
int32 bullets_allowance

# 队友位置
float32 hero_pose_x
float32 hero_pose_y
float32 infantry_pose_x
float32 infantry_pose_y

# 联盟赛专用
# 中心增益点占领情况
int32 center_status
# 剩余金币
int32 remain_gold
