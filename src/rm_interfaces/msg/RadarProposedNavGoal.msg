# 雷达 -> 哨兵：提议导航目标（项目坐标系，与黑板 @nav_goal 一致）
# 整型秒（替代 builtin_interfaces/Time stamp；语义由雷达/哨兵约定）
int32 time_sec
# 项目坐标系雷达建议目标位置
float32 radar_goal_x
float32 radar_goal_y
# 雷达建议姿态（stance 编号，-1 表示未指定）
int32 radar_advice_stance
# 比赛开始为 0，雷达端建议每秒 +1，用于哨兵侧链路活性判断
uint32 time_go
