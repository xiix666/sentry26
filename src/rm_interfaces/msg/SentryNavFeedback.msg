# 哨兵 -> 雷达：生效导航目标与重选点反馈（项目坐标系）
float32 current_x
float32 current_y

int32 bullets_allowance
# 项目坐标系平面位置（原 geometry_msgs/PoseStamped effective_goal）
float32 effective_goal_x
float32 effective_goal_y

bool replanned_by_costmap
# 是否在追击中：与哨兵黑板 @lock_pursuit_ing 一致（lock_enemy_id!=0 且 @sentry_pursuit_enable）
bool lock_pursuit_ing
# 锁敌状态：与哨兵黑板 @sentry_lock_enemy_id 一致；0 表示未锁敌（语义同 rm_interfaces/SentryLockEnemy）
int32 lock_enemy_id
# 回传最近一次收到的 RadarProposedNavGoal.time_go，供雷达端核对
uint32 time_back

# 以下为相机链路镜像；槽位顺序：英雄、工程、步兵1、步兵2、敌方哨兵
# enemy_id[i]：检出则为固定兵种编号 1/2/3/4/7，未检出为 0（坐标同槽填 0）
int32 enemy_count
int32[5] enemy_id
float32[5] enemy_x
float32[5] enemy_y
