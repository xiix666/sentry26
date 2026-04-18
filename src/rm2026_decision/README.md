# RM2026 机器人决策系统

## 项目概述

这是一个基于 ROS2 和 BehaviorTree.CPP 的 RM2026 机器人竞赛哨兵机器人决策系统。该系统通过行为树实现智能决策逻辑，能够根据比赛状态、敌方位置和己方状态自主进行导航、巡逻和战术决策。

## 核心架构

### 技术栈
- **ROS2 Humble**: 机器人操作系统框架
- **BehaviorTree.CPP**: 行为树决策引擎
- **C++17**: 核心实现语言
- **多线程架构**: 保证实时性决策

## 文件结构

```
src/
├── rm2026_decision/                    # 主决策包
│   ├── CMakeLists.txt                  # 构建配置
│   ├── package.xml                     # ROS2包配置
│   ├── src/
│   │   ├── decision_node.cpp           # 主决策节点
│   │   └── behavior_nodes.cpp          # 行为节点注册
│   ├── behaviors/                      # 行为节点实现
│   │   ├── combat_behaviors.hpp        # 战斗相关行为
│   │   ├── Go*.hpp                     # 导航行为节点
│   │   ├── *Patrol*.hpp                # 巡逻行为节点
│   │   ├── NavigateToSupplyPoint.hpp   # 补给点导航
│   │   ├── RetreatToLowerArea.hpp      # 撤退行为
│   │   ├── StayAtSupplyPoint.hpp       # 补给点停留
│   │   ├── HeroWhere.hpp               # 英雄位置判断
│   │   └── WhereArea.hpp               # 区域判断
│   ├── conditions/                     # 条件节点实现
│   │   └── Area2YellowTimer.hpp        # 区域2黄色点停留计时器
│   ├── config/                         # 行为树配置
│   │   ├── Area*.xml                   # 各区域行为树
│   │   ├── AreaMain.xml                # 主行为树
│   │   └── decision.btproj             # Groot编辑器项目
│   └── launch/
│       └── rm2026_decision.launch.py    # 启动文件
└── rm2026_interfaces/                   # 接口定义包
    ├── CMakeLists.txt
    ├── package.xml
    ├── msg/
    │   ├── CameraMsg.msg               # 相机数据消息
    │   ├── ReceiveMsg.msg              # 裁判系统消息
    │   ├── NavMsg.msg                  # 导航消息
    │   └── AngleMsg.msg                # 角度消息
```

## 核心算法介绍

### 1. 行为树决策系统

系统采用分层行为树架构：

#### 主决策树 (AreaMain.xml)
```
位置判断 → 区域选择 → 执行对应区域策略
```

#### 区域决策树
- **Area0Tree**: 未知区域处理
- **Area1Tree**: 区域1策略（血量检查→补给或前进）
- **Area2Tree**: 区域2策略（包含黄色区域定时器）
- **Area3Tree**: 区域3巡逻策略
- **Area4Tree**: 区域4策略（包含红蓝区域区分）
- **Area5Tree**: 边界区域处理

### 2. 智能导航算法

#### 位置感知与坐标变换
- **TF2监听**: 实时获取机器人位置 (`base_link` → `odom`)
- **队伍对称处理**: 红蓝方自动进行中心对称坐标变换
- **全局黑板**: 共享状态信息，避免重复计算

#### 导航行为节点
- **NavigateToSupplyPoint**: 导航至补给点
- **GoToArea[1-4]**: 区域间导航
- **StayAtSupplyPoint**: 在补给点停留

### 3. 巡逻算法

#### 多点巡逻策略
- **Area1Patrol3**: 区域1三点巡逻 (1250,5700) → (840,3900) → (1700,3900)
- **Area3Patrol3**: 区域3三点巡逻策略
- **Area2YellowTimer**: 区域2黄色区域定时停留

#### 巡逻特点
- 循环遍历预设巡逻点
- 支持中断和恢复
- 考虑敌方威胁评估

### 4. 敌方检测与跟踪算法

#### 多相机融合
```cpp
// 4个相机数据融合，优先级: 相机1 > 相机2 > 相机3 > 相机4
std::array<EnemyDetection, 5> enemy_detections; // 5种兵种
```

#### 车辆类型识别
- **英雄 (hero)**: ID索引0
- **工程 (engineer)**: ID索引1  
- **步兵1 (infantry1)**: ID索引2
- **步兵2 (infantry2)**: ID索引3
- **哨兵 (sentry)**: ID索引4

#### 运动预测
- 实时速度估计 (speed_x, speed_y)
- 位置跟踪与更新

### 5. 战术决策算法

#### 血量管理策略
```xml
<!-- 血量低于300时前往补给点 -->
<Precondition if="@sentry_hp &lt; 300">
    <StayAtSupplyPoint/>
</Precondition>
```

#### 区域战术
- **区域1**: 补给检查 → 前往区域2
- **区域2**: 黄色区域停留战术
- **区域3**: 巡逻 + 增益点控制
- **区域4**: 分区域战术（红区/蓝区/黄区）

#### 撤退机制
- **RetreatToLowerArea**: 向区域数减少方向撤退
- 基于当前区域ID的智能撤退路径选择

### 6. 状态管理算法

#### 全局黑板系统
- **位置信息**: 当前位置、导航点、队友位置
- **比赛状态**: 游戏进度、剩余时间、队伍颜色
- **己方状态**: 血量、弹药、功率限制
- **敌方信息**: 检测状态、位置、速度

#### 实时数据更新
- **相机数据**: 10Hz更新频率
- **裁判系统**: 实时比赛状态同步
- **TF位置**: 10Hz位置更新

## 使用方法

### 1. 环境依赖
```bash
# ROS2 Humble
# behaviortree_cpp >= 4.0
# nav2 相关包
```

### 2. 构建
```bash
cd /path/to/workspace
colcon build --packages-select rm2026_decision rm2026_interfaces
```

### 3. 运行
```bash
# 启动决策节点
ros2 launch rm2026_decision rm2026_decision.launch.py

# 或直接运行节点
ros2 run rm2026_decision rm2026_decision_node
```

### 4. 参数配置
```yaml
# launch文件中的参数
tree_file: "HighPTree.xml"          # 行为树文件
tree_directory: "config"            # 树文件目录
tick_frequency: 10.0                # 执行频率(Hz)
```

## 接口说明

### 输入话题
- `/camera_topic` (CameraMsg): 相机检测数据
- `/receiveLLC_pack` (ReceiveMsg): 裁判系统数据

### 输出话题
- `navigation/goal` (PoseStamped): 导航目标点
- `navigation/status` (Bool): 导航完成状态

### 关键参数
- `team_color`: 队伍颜色 (0=蓝方, 1=红方)
- `current_area`: 当前所在区域 (0-5)
- `sentry_hp`: 哨兵血量
- `enemy_count`: 检测到的敌人数量

## 扩展开发

### 添加新行为节点
1. 在 `behaviors/` 目录创建新的 `.hpp` 文件
2. 继承 `BT::StatefulActionNode` 或 `BT::SyncActionNode`
3. 在 `behavior_nodes.cpp` 中注册节点
4. 在行为树XML中配置使用

### 修改决策逻辑
1. 使用 Groot 编辑器打开 `decision.btproj`
2. 修改对应区域的行为树
3. 重新构建并测试

## 性能特性

- **实时性**: 10Hz决策频率，支持高速运动场景
- **可靠性**: 多线程架构，异常安全处理
- **可扩展性**: 模块化设计，易于添加新行为
- **鲁棒性**: 超时机制、状态检查、错误恢复

## 注意事项

1. **坐标系**: 使用map坐标系，所有位置以米为单位
2. **队伍对称**: 系统自动处理红蓝方坐标变换
3. **并发安全**: 全局黑板线程安全，支持多节点并发访问
4. **资源管理**: 自动清理ROS订阅者和发布者，避免资源泄漏