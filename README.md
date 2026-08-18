<p align="center">
  <img src="asset/视觉.png" alt="RPS 战队队徽" width="240">
</p>

# RPS舵轮哨兵导航系统

## 项目介绍

  本项目是一个基于Nav2框架、面向RoboMaster（RM）哨兵机器人场景的ROS 2导航
  项目。项目以二维代价地图为基础，在有限算力下完成定位、感知、规划与控制，并
  重点提高机器人通过起伏路段、坡面和下台阶时的稳定性。本项目由我作为RPS战队
  成员进行开发，并用于战队哨兵机器人的导航系统。

  项目面向单计算设备同时运行两台雷达、四台相机和完整导航链路的实际需求，将
  轻量化和稳定性作为主要设计目标。定位部分对Point-LIO做了较多工程和性能优化；
  规划控制部分没有采用近期RM中常见的MINCO + MPC多项式轨迹架构，而是保留离散
  路径表示，并对规划后端、路径安全性和跟踪控制进行优化。

  由于RM比赛场地和目标地形相对固定，项目中的部分地形识别与跨越逻辑采用了针对
  具体场景的参数和规则，部分处理甚至是直接写死的。这种实现牺牲了一定的通用性，
  但能够减少计算开销和不确定性，以更直接的方式保证实车运行的稳定性。因此，本
  项目目前更适合作为RM场景下的工程实践，而不是开箱即用的通用地形导航方案。

  当前版本在第13代Intel Core i7 NUC上仅运行导航链路时，观察到的CPU占用率低于
  10%。该数据是当前软硬件配置下的实测参考，并非标准化benchmark，实际结果会
  受到传感器频率、参数和编译选项影响。

  这个项目从本赛季开始开发，至今不足一年。在开始开发前，我没有ROS 2使用经验，
  项目也几乎完全从零搭建；开发期间还同时参与了队内其他项目。因此，当前代码在
  架构、接口、参数配置和工程完整性等方面仍有不少不成熟之处，也可能存在只适用于
  当前机器人和赛场配置的实现。请将它视为一个仍在快速迭代的工程项目，而不是成熟
  完善的最终方案。后续我会结合实车测试和比赛需求持续维护、补充文档并改进实现。

  系统围绕哨兵机器人的定位、导航、场地感知和底盘通信组织完整链路。在Nav2二维
  导航框架中，项目通过地形分析和定制代价地图层表达场地中的高程变化与特殊区域。
  RViz、建图、重定位、地图保存和全向感知均可按需启停，调试配置与实车配置相互
  独立。

  模块职责和数据流的详细说明见[docs/overview.md](docs/overview.md)。

## 项目结构

以下结构列出当前开源源码中的主要目录和模块。

```text
.
├── README.md
├── docs/
│   └── overview.md                  # 模块职责与数据流
├── start/                           # 构建、启动和 systemd 示例脚本
└── src/
    ├── bringup/                     # Launch、参数、地图、RViz 与行为树
    ├── rm_hardware_driver/
    │   ├── lidar_output/            # 双雷达数据融合
    │   └── livox_ros_driver2_humble/# Livox ROS 2 驱动
    ├── rm_interfaces/               # 项目自定义 ROS 2 消息
    ├── rm_localization/
    │   ├── point_lio/               # 优化后的 Point-LIO
    │   ├── small_gicp_relocalization/# 基于 small_gicp 的重定位
    │   └── map_save/                # 周期性地图保存
    ├── rm_navigation/
    │   ├── theta_star_planner/      # 全局离散路径规划
    │   ├── pb_omni_pid_pursuit_controller/ # 全向底盘路径跟踪
    │   ├── constraint_vel_controller/# 速度与运动约束
    │   ├── fake_vel_transform/      # 速度及坐标系适配
    │   └── move_to_free/            # 脱困恢复行为
    ├── rm_perception/
    │   ├── terrain/                 # 地形分析与高程障碍点云
    │   ├── pointcloud_frame_trans/  # 点云和里程计坐标适配
    │   ├── pc2scan/                 # 点云转二维激光扫描
    │   ├── voxel_obstacle/          # 定制体素障碍层
    │   ├── static_xx_layer/         # 比赛区域静态代价地图层
    │   ├── special_area/            # 特殊场地区域处理
    │   └── omni_perception/         # 全向感知信息适配
    └── rm_port/
        └── rm_serial_driver-main/   # 导航系统与底盘控制器通信
```

## 启动配置

项目保留三个相互独立的顶层启动文件，便于针对调试和实车部署分别增删模块：

| 启动文件 | 用途 | 默认参数文件 |
|---|---|---|
| `c_reloc_bringup_launch.py` | rosbag 与日常调试 | `nav_params_1.yaml` |
| `nav_launch.py` | 实车开机自启主配置 | `nav_params.yaml` |
| `nav2_launch.py` | 实车备用配置 | `nav_params_1.yaml` |

主要可选参数：

| 参数 | 作用 |
|---|---|
| `mapping_mode` | 是否启动 `pc2scan` 和 `slam_toolbox` 建图链路 |
| `relocate` | 是否启动 `small_gicp_relocalization` |
| `use_rviz` | 是否启动 RViz |
| `use_omni_perception` | 是否启动全向感知模块 |
| `use_map_save` | 是否周期性保存地图 |
| `use_sim_time` | 是否使用 `/clock` 仿真时间 |

## 环境与构建

当前开发环境：

- Ubuntu 22.04
- ROS 2 Humble
- CMake / colcon
- Livox MID-360 系列雷达

克隆后安装依赖：

```bash
cd ~/sentry26
rosdep install --from-paths src --ignore-src -r -y
```

编译：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 运行

日常调试：

```bash
ros2 launch bringup c_reloc_bringup_launch.py
```

实车导航：

```bash
ros2 launch bringup nav_launch.py
```

按需组合功能：

```bash
ros2 launch bringup nav_launch.py \
  mapping_mode:=false \
  relocate:=true \
  use_rviz:=true \
  use_omni_perception:=false \
  use_map_save:=false
```

使用 rosbag 时需要播放时钟，并确保启动文件启用仿真时间：

```bash
ros2 bag play <bag-path> --clock
ros2 launch bringup c_reloc_bringup_launch.py use_sim_time:=true
```

## 开源说明

本仓库包含来自 Point-LIO、Livox ROS Driver 2 等项目的代码。相关目录保留各自
的许可证和作者信息。

根目录统一开源许可证、第三方依赖清单和贡献指南仍在整理中。在正式发布前，
请补充根目录 `LICENSE`，并再次核对地图、配置、二进制库和比赛相关资源的发布
权限。
