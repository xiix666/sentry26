# 裁判系统通信消息（电控发的消息）
# 话题：/receive_pack

# 比赛信息
# 当前比赛状态
int32 game_progress
# 比赛剩余时间
int32 stage_remain_time
# 红蓝方（红方为1,蓝方为0）
int32 red_blue

# 己方信息
# 己方机器人血量
int32 self_1_hp
int32 self_2_hp
int32 self_3_hp
int32 self_4_hp
int32 self_7_hp
# 己方前哨站血量
int32 self_outpost_hp
# 己方基地血量
int32 self_base_hp
# 允许发弹量
int32 bullets_allowance
# 底盘功率
float32 chassis_power
# 己方小能量机关激活状态
int32 self_small_energy
# 己方大能量机关激活状态
int32 self_big_energy
# 哨兵累积发弹量
int32 total_bullets
# 操作手坐标
float32 operator_x
float32 operator_y

# 敌方信息
# 敌方机器人血量
int32 post_1_hp
int32 post_2_hp
int32 post_3_hp
int32 post_4_hp
int32 post_7_hp
# 敌方前哨站血量
int32 post_outpost_hp
# 敌方基地血量
int32 post_base_hp
# 对方飞镖最后一次击中己方前哨站或基地的时间
int32 dart_time
# 对方飞镖最后一次击中己方前哨站或基地的具体目标
int32 dart_type

# 剩余金币
int32 remain_gold

# 20000J热量是否用完
int32 heat_flag

# 决策代号
int32 decision_code