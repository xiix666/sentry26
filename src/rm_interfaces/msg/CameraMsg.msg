# 发给自瞄的消息
# 话题：/camera_topic

std_msgs/Header header
# 相机1（枪管上的主相机）
# 前车ID
int32[5] id_1
# 前车位置
float32[5] enemy_pose_1_x
float32[5] enemy_pose_1_y
float32[5] enemy_pose_1_z

# 相机2（全向感知相机1）
# 前车ID
int32[5] id_2
# 前车位置
float32[5] enemy_pose_2_x
float32[5] enemy_pose_2_y
float32[5] enemy_pose_2_z

# 相机3（全向感知相机2）
# 前车ID
int32[5] id_3
# 前车位置
float32[5] enemy_pose_3_x
float32[5] enemy_pose_3_y
float32[5] enemy_pose_3_z

# 相机3（全向感知相机3）
# 前车ID
int32[5] id_4
# 前车位置
float32[5] enemy_pose_4_x
float32[5] enemy_pose_4_y
float32[5] enemy_pose_4_z

# 前车移动预测（x方向速度，y方向速度）
float32[5] speed_x
float32[5] speed_y

# 车辆顺序：英雄、工程、步兵1、步兵2、哨兵
