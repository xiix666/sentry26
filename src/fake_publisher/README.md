# Fake Publisher - RM2026 假数据发布器

这是一个用于测试 RM2026 决策系统的假数据发布器，可以模拟相机数据、裁判系统数据和导航状态反馈。

## 功能特性

- **相机数据发布**：模拟敌方车辆位置和速度信息
- **裁判系统数据发布**：模拟比赛状态、己方血量、弹药等信息
- **导航状态反馈**：响应导航目标并反馈完成状态
- **可配置参数**：支持运行时参数调整
- **多线程设计**：支持并发数据发布

## 发布话题

### 输出话题
- **`camera_topic`** (CameraMsg)
  - 模拟相机检测到的敌方车辆信息
  - 包含车辆ID、位置、速度等数据

- **`receiveLLC_pack`** (ReceiveMsg)
  - 模拟裁判系统发送的比赛数据
  - 包含比赛状态、己方血量、弹药等信息

- **`navigation/status`** (Bool)
  - 导航完成状态反馈
  - 响应导航目标后延迟发布完成信号

### TF变换广播
- **map → base_link**
  - 发布机器人位姿变换
  - 支持 decision_node.cpp 中的 TF 查询
  - 频率：50Hz

### 订阅话题
- **`navigation/goal`** (PoseStamped)
  - 接收导航目标点
  - 触发导航状态反馈

## 使用方法

### 1. 构建
```bash
cd /path/to/workspace
colcon build --packages-select fake_publisher rm2026_interfaces
```

### 2. 运行
```bash
# 直接运行节点
ros2 run fake_publisher fake_publisher_node

# 或使用launch文件
ros2 launch fake_publisher fake_publisher.launch.py
```

### 3. 与决策系统联调
```bash
# 终端1：启动假数据发布器
ros2 launch fake_publisher fake_publisher.launch.py

# 终端2：启动决策系统
ros2 launch rm2026_decision rm2026_decision.launch.py
```

## 相机数据配置

相机数据直接在 `publishCameraData()` 函数中配置。每个相机都设置了所有5种敌人的位置信息：

```cpp
// 每个相机都包含所有5种敌人类型的位置
// 相机1配置 - 主要检测区域
msg->enemy_pose_1[0].pose.position.x = 8.0;   // 英雄位置
msg->enemy_pose_1[0].pose.position.y = 6.0;

msg->enemy_pose_1[1].pose.position.x = 7.5;   // 工程位置
msg->enemy_pose_1[1].pose.position.y = 5.5;

msg->enemy_pose_1[2].pose.position.x = 7.0;   // 步兵1位置
msg->enemy_pose_1[2].pose.position.y = 5.0;

// ... 相机2、相机3、相机4 类似配置

// 全局速度设置（所有相机同类型敌人共享）
msg->speed_x[0] = -1.2;   // 所有英雄向左移动
msg->speed_y[0] = 0.0;

msg->speed_x[4] = 1.5;    // 所有哨兵快速右移
msg->speed_y[4] = -0.8;   // 同时向下移动
```

**速度配置说明**:
- 速度数组是全局的，所有相机的同类型敌人共享相同的速度
- 例如：设置 `msg->speed_x[0] = -1.2` 会让所有相机的英雄都向左移动
- 速度单位：m/s，0.0表示静止

### TF变换配置
机器人位置直接在 `publishTFTransform()` 函数中配置：

```cpp
// 机器人位姿设置
robot_x_ = 6.0;     // X位置 (米)
robot_y_ = 4.0;     // Y位置 (米)
robot_yaw_ = 0.0;   // 朝向角度 (弧度)
```

**TF变换说明**:
- 发布 `map` → `base_link` 的坐标变换
- 支持 decision_node.cpp 中的 TF 查询：`lookupTransform("map", "base_link")`
- 频率：50Hz，确保实时性
- 初始位置可在launch文件中配置

**数组索引对应关系**:
- 0: hero (英雄)
- 1: engineer (工程)
- 2: infantry1 (步兵1)
- 3: infantry2 (步兵2)
- 4: sentry (哨兵)

**当前配置概览**:
- **相机1**: 主要区域 - 所有5种敌人的位置坐标
- **相机2**: 左侧区域 - 所有5种敌人的位置坐标
- **相机3**: 右侧区域 - 所有5种敌人的位置坐标
- **相机4**: 后方区域 - 所有5种敌人的位置坐标

### 裁判系统参数
裁判系统数据直接在 `publishRefereeData()` 函数中配置：

```cpp
// 比赛信息
msg->game_progress = 4;      // 比赛进行中
msg->stage_remain_time = 300; // 5分钟
msg->red_blue = 0;           // 蓝方

// 己方血量
msg->self_hero_hp = 400;     // 英雄满血
msg->self_infantry_hp = 300; // 步兵满血
msg->self_sentry_hp = 400;   // 哨兵满血

// 弹药和功率
msg->bullets_allowance = 50; // 弹药限额
msg->chassis_power = 30.0;   // 底盘功率
```

### 导航参数
导航响应延迟直接在 `handleNavigationGoal()` 函数中设置：

```cpp
double delay = 1.0;  // 1秒延迟响应导航目标
```

## 运行时参数修改

可以通过命令行动态修改参数：

```bash
# 修改导航响应延迟
ros2 param set /fake_publisher_node navigation_response_delay 2.0

# 注意：裁判系统数据现在直接在代码中设置，不再支持运行时参数修改
# 要修改裁判系统数据，请直接修改 publishRefereeData() 函数中的值
```

**注意**: 相机数据现在直接在代码中修改，不再支持运行时参数修改。

## 测试场景示例

### 场景1：敌方接近测试
```cpp
// 在 publishCameraData() 函数中修改：

// 设置相机1检测到的英雄靠近哨兵
msg->enemy_pose_1[0].pose.position.x = 2.0;  // 英雄靠近
msg->enemy_pose_1[0].pose.position.y = 3.0;
msg->speed_x[0] = -2.0;  // 英雄快速向左移动

// 设置相机2检测到的步兵快速接近
msg->enemy_pose_2[2].pose.position.x = 1.5;
msg->enemy_pose_2[2].pose.position.y = 3.5;
msg->speed_x[2] = -2.5;  // 步兵1快速向左移动

// 设置相机3的哨兵高速接近
msg->enemy_pose_3[4].pose.position.x = 4.0;  // 哨兵靠近
msg->enemy_pose_3[4].pose.position.y = 3.0;
msg->speed_x[4] = -3.0;  // 哨兵高速向左移动
msg->speed_y[4] = 0.0;   // 停止向下移动
```

### 场景2：血量不足测试
```cpp
// 在 publishRefereeData() 函数中修改：

// 设置血量过低
msg->self_sentry_hp = 250;   // 哨兵血量降低
msg->self_hero_hp = 150;     // 英雄血量降低
```

### 场景3：弹药不足测试
```cpp
// 在 publishRefereeData() 函数中修改：

// 设置弹药不足
msg->bullets_allowance = 40; // 弹药限额降低
```

## 日志输出

节点会定期输出关键信息：
```
[INFO] TF广播: map -> base_link (6.00, 4.00, 0.00)
[INFO] 相机数据发布:
  相机1: hero(8.0,6.0), engineer(7.5,5.5), infantry1(7.0,5.0), infantry2(6.5,4.5), sentry(6.0,4.0)
  相机2: hero(2.0,7.0), engineer(1.5,6.5), infantry1(1.0,6.0), infantry2(3.0,6.0), sentry(2.5,5.5)
  相机3: hero(9.0,5.0), engineer(9.5,4.5), infantry1(10.0,4.0), infantry2(10.5,3.5), sentry(11.0,3.0)
  相机4: hero(6.0,0.5), engineer(5.5,0.8), infantry1(6.5,0.8), infantry2(7.0,0.5), sentry(5.0,1.0)
[INFO] 发布裁判系统数据: 队伍=蓝方, 血量=400/300/400, 弹药=50
[INFO] 收到导航目标: (6.00, 4.00)
[INFO] 导航完成状态已发布
```

## 代码设计特点

### 直接代码配置设计
- **代码内配置**：相机数据直接在 `publishCameraData()` 函数中设置
- **快速修改**：无需参数声明，直接修改代码即可
- **类型安全**：编译时检查，避免运行时错误
- **版本控制友好**：配置变更可以追踪

### 智能日志输出
- **结构化显示**：按相机分组显示检测结果
- **实时反馈**：定期输出当前检测状态
- **调试友好**：便于观察和分析系统行为

## 注意事项

1. **坐标系**：所有位置坐标使用地图坐标系（米为单位）
2. **位置设置**：直接在代码中设置敌人的位置坐标
3. **队伍对称**：系统会根据队伍颜色自动处理坐标变换
4. **导航反馈**：收到导航目标后会延迟1秒发送完成状态
5. **多线程安全**：节点使用多线程执行器，支持并发操作
6. **参数验证**：修改参数时注意ID、坐标和速度的合理性