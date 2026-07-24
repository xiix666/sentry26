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
int32 self_hero_hp
int32 self_engineer_hp
int32 self_infantry3_hp
int32 self_infantry4_hp
int32 self_sentry_hp
int32 self_outpost_hp
# 己方基地血量
int32 self_base_hp
# 敌方前哨站血量
int32 enemy_outpost_hp

# 哨兵位置
float32 self_pose_x
float32 self_pose_y
# 允许发弹量
int32 bullets_allowance

# 己方机器人位置（单位：m，场地坐标系）
# 己方英雄机器人位置
float32 self_hero_pose_x
float32 self_hero_pose_y
# 己方工程机器人位置
float32 self_engineer_pose_x
float32 self_engineer_pose_y
# 己方 3 号步兵机器人位置
float32 self_infantry3_pose_x
float32 self_infantry3_pose_y
# 己方 4 号步兵机器人位置
float32 self_infantry4_pose_x
float32 self_infantry4_pose_y

# 联盟赛专用
# 剩余金币
int32 remain_gold

# 增益状态
# 回血增益
int32 recovery_buff
# 防御增益
int32 defence_buff
# 防御 debuff
int32 defence_debuff
# 攻击增益
int32 attack_buff
# 后3bit 占堡垒 中镖（1中） 
int32 status_info