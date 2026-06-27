# ROS2 小车项目学习指南

这份文档用于学习当前 ROS2 小车项目的整体结构、运行流程和关键参数。重点不是一次性记住所有配置，而是知道数据从哪里来、经过哪些节点、最后怎样变成建图和导航行为。

当前项目的主线是：

1. 底盘 ESP32 通过串口接入 ROS2，发布 `/odom`、`/imu/data`，接收 `/cmd_vel`。
2. YDLidar 发布 `/scan`。
3. `tf_tree_node` 发布 TF，让 ROS2 知道 `odom`、`base_link`、`laser_link` 等坐标系关系。
4. `slam_gmapping` 使用 `/scan`、`/odom`、TF 实时生成 `/map`，并发布 `map -> odom`。
5. Nav2 使用实时 `/map`、`/scan`、`/odom` 规划路径，输出 `/cmd_vel` 控制小车。

## 1. 工作区结构

```text
ros_car/
├── docs/
│   ├── ros2_car_build_and_mapping_steps.md
│   └── ros2_car_project_learning_guide.md
├── src/
│   ├── my_pkg/
│   │   ├── launch/
│   │   │   ├── bringup.launch.py
│   │   │   ├── live_mapping.launch.py
│   │   │   ├── live_navigation.launch.py
│   │   │   └── navigation.launch.py
│   │   ├── config/
│   │   │   ├── tf_params.yaml
│   │   │   └── nav2_params.yaml
│   │   └── my_pkg/
│   │       ├── esp32_bridge/
│   │       ├── keyboard_control/
│   │       └── tf_tree/
│   ├── ydlidar_ros2_driver/
│   │   ├── launch/
│   │   └── params/ydlidar.yaml
│   ├── slam_gmapping/
│   │   ├── launch/
│   │   └── params/slam_gmapping.yaml
│   └── openslam_gmapping/
├── src/xc_urdf/
│   ├── launch/display.launch.py
│   ├── urdf/xc_urdf.urdf
│   └── meshes/
└── XC-URDF/
    └── 原始 SolidWorks 导出的 ROS1/catkin 包，仅作为源文件参考
```

几个包的职责：

| 包 | 作用 |
| --- | --- |
| `my_pkg` | 你的主控包，负责底盘桥接、TF、键盘控制、Nav2 启动配置 |
| `ydlidar_ros2_driver` | 雷达驱动包，发布标准 `sensor_msgs/LaserScan` 到 `/scan` |
| `slam_gmapping` | ROS2 版本 GMapping 节点，实时发布 `/map` |
| `openslam_gmapping` | GMapping 底层算法库，被 `slam_gmapping` 调用 |
| `xc_urdf` | ROS2 可用的小车 URDF 描述包，用于 RViz2 显示车体模型 |
| `XC-URDF` | 原始 SolidWorks 导出的 ROS1/catkin 包，已用 `COLCON_IGNORE` 避免被 ROS2 构建 |

## 2. 推荐启动顺序

实时建图导航建议分三步启动。这样出问题时能快速定位是哪一层没有工作。

### 第一步：启动底盘、雷达和 TF

```bash
cd ~/ros2_car
source install/setup.bash
ros2 launch my_pkg bringup.launch.py use_lidar:=true
```

这个 launch 会启动：

| 节点 | 来自 | 作用 |
| --- | --- | --- |
| `esp32_bridge_node` | `my_pkg` | 串口连接 ESP32，收编码器/IMU，发轮速命令 |
| `tf_tree_node` | `my_pkg` | 发布 `odom -> base_link` 动态 TF 和雷达等静态 TF |
| `ydlidar_ros2_driver_node` | `ydlidar_ros2_driver` | 发布 `/scan` |

验证：

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 topic echo /imu/data --once
ros2 run tf2_ros tf2_echo base_link laser_link
```

### 第二步：启动实时建图

```bash
ros2 launch my_pkg live_mapping.launch.py
```

这个 launch 只启动 GMapping。它会订阅 `/scan`，读取 TF 中的 `odom -> base_link -> laser_link`，然后发布：

| 输出 | 作用 |
| --- | --- |
| `/map` | 实时占据栅格地图 |
| `/map_metadata` | 地图分辨率、尺寸、原点 |
| `map -> odom` | 建图算法计算出的全局校正 TF |

验证：

```bash
ros2 topic echo /map_metadata --once
ros2 run tf2_ros tf2_echo map odom
```

### 第三步：启动实时导航

```bash
ros2 launch my_pkg live_navigation.launch.py use_rviz:=true
```

这个 launch 不启动 `map_server` 和 AMCL，它适合实时建图时导航。地图来源是 GMapping 正在发布的 `/map`。

如果你要使用保存好的地图导航，则运行：

```bash
ros2 launch my_pkg navigation.launch.py map:=$HOME/ros2_car/maps/first_map.yaml use_rviz:=true
```

实时建图导航和保存地图导航的区别：

| 模式 | 地图来源 | 定位来源 | 适合场景 |
| --- | --- | --- | --- |
| `live_mapping.launch.py` + `live_navigation.launch.py` | GMapping 实时 `/map` | GMapping 发布 `map -> odom` | 边建图边导航 |
| `navigation.launch.py` | 已保存地图文件 | AMCL 发布 `map -> odom` | 地图已稳定，重复导航 |

## 3. 核心话题

| 话题 | 类型 | 谁发布 | 谁使用 | 作用 |
| --- | --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 或键盘控制 | `esp32_bridge_node` | 小车速度命令 |
| `/odom` | `nav_msgs/Odometry` | `esp32_bridge_node` | TF、Nav2、GMapping | 里程计位置和速度 |
| `/imu/rawdata` | `sensor_msgs/Imu` | `esp32_bridge_node` | 调试 | 原始 IMU 数据 |
| `/imu/data` | `sensor_msgs/Imu` | `esp32_bridge_node` | 里程计融合/调试 | 修正后的 IMU 姿态 |
| `/scan` | `sensor_msgs/LaserScan` | YDLidar | GMapping、Nav2 costmap | 激光雷达扫描 |
| `/map` | `nav_msgs/OccupancyGrid` | GMapping 或 map_server | Nav2、RViz | 地图 |
| `/plan` | `nav_msgs/Path` | Nav2 planner | RViz、controller | 全局路径 |
| `/local_plan` | `nav_msgs/Path` | Nav2 controller | RViz | 局部轨迹 |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | Nav2 | RViz、planner | 全局代价地图 |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | Nav2 | RViz、controller | 局部代价地图 |

## 4. TF 坐标系

当前项目最重要的 TF 链路是：

```text
map
└── odom
    └── base_link
        ├── base_footprint
        ├── imu_link
        ├── laser_link
        └── camera_link
```

每段 TF 的来源：

| TF | 来源 | 含义 |
| --- | --- | --- |
| `map -> odom` | GMapping 或 AMCL | 全局定位修正 |
| `odom -> base_link` | `tf_tree_node` 从 `/odom` 转发 | 小车相对起点的运动 |
| `base_link -> laser_link` | `tf_tree_node` 静态 TF | 雷达相对车体的位置 |
| `base_link -> imu_link` | `tf_tree_node` 静态 TF | IMU 相对车体的位置 |

如果建图或导航报 TF 错误，先查这几条：

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_link
ros2 run tf2_ros tf2_echo map odom
```

## 5. `bringup.launch.py` 参数

文件：`src/my_pkg/launch/bringup.launch.py`

这个文件负责启动底盘、TF、可选键盘、可选雷达、可选 GMapping。你平时最常改的是串口、轮距、IMU 方向和是否启动雷达。

| 参数 | 当前默认值 | 修改效果 |
| --- | --- | --- |
| `port` | `/dev/ttyUSB1` | ESP32 底盘串口。改错会导致底盘连不上 |
| `baudrate` | `115200` | ESP32 串口波特率，必须和下位机一致 |
| `wheel_base_m` | `0.18` | 左右轮距离。影响 `/cmd_vel` 转左右轮速度，也影响差速转弯 |
| `enable_print` | `false` | 打开后会周期打印编码器、IMU、速度命令，调试时有用 |
| `odom_publish_rate_hz` | `20.0` | `/odom` 发布频率。太高占 CPU，太低导航延迟 |
| `use_imu_yaw_for_odom` | `true` | 是否用 IMU yaw 修正 `/odom` 朝向 |
| `imu_yaw_sign` | `-1.0` | 修正 IMU yaw 正方向。左转显示右转时改这个 |
| `use_keyboard_control` | `false` | 是否启动键盘控制 |
| `use_lidar` | `false` | 是否同时启动雷达驱动 |
| `lidar_port` | `/dev/ydlidar` | 雷达串口 |
| `use_gmapping` | `false` | 是否在 bringup 里一起启动 GMapping。现在建议单独用 `live_mapping.launch.py` |

示例：

```bash
# 启动底盘和雷达
ros2 launch my_pkg bringup.launch.py use_lidar:=true

# 如果 IMU 方向仍然反了，切回原方向测试
ros2 launch my_pkg bringup.launch.py use_lidar:=true imu_yaw_sign:=1.0

# 调试串口数据时打开打印
ros2 launch my_pkg bringup.launch.py use_lidar:=true enable_print:=true
```

## 6. 底盘桥接节点

文件：`src/my_pkg/my_pkg/esp32_bridge/esp32_bridge_node.py`

`esp32_bridge_node` 是 ROS2 和 ESP32 的桥。它做四件事：

1. 订阅 `/cmd_vel`。
2. 把 `linear.x` 和 `angular.z` 换算成左右轮速度。
3. 通过串口发送给 ESP32。
4. 从 ESP32 接收编码器和 IMU，发布 `/odom`、`/imu/rawdata`、`/imu/data`。

差速小车速度换算公式：

```text
left_speed  = linear_speed - angular_speed * wheel_base_m / 2
right_speed = linear_speed + angular_speed * wheel_base_m / 2
```

参数影响：

| 参数 | 修改效果 |
| --- | --- |
| `wheel_base_m` 变大 | 同样角速度下，左右轮速度差变大，原地转弯更用力 |
| `wheel_base_m` 变小 | 同样角速度下，左右轮速度差变小，可能转不动或转得慢 |
| `MAX_WHEEL_SPEED_CM_S` | 限制发给 ESP32 的最大轮速，保护电机 |
| `imu_yaw_sign` | 同时修正 `/odom` 姿态和 `/imu/data` 里的 yaw、gyro_z |
| `use_imu_yaw_for_odom` | 关闭后只用编码器差速积分 yaw，容易受轮胎打滑/左右轮误差影响 |

当前最大轮速限制在代码中：

```python
MAX_WHEEL_SPEED_CM_S = 50.0
```

如果 Nav2 给的速度很大，最终发给 ESP32 的左右轮速度也不会超过正负 `50 cm/s`。

## 7. TF 参数

文件：`src/my_pkg/config/tf_params.yaml`

当前参数：

```yaml
laser_x: 0.0
laser_y: 0.0
laser_z: 0.0
```

这些值表示雷达相对 `base_link` 的安装位置，单位是米。

| 参数 | 正方向 | 修改效果 |
| --- | --- | --- |
| `laser_x` | 车头方向为正 | 雷达在车体前方就设正值，例如 `0.08` |
| `laser_y` | 左侧为正 | 雷达偏左为正，偏右为负 |
| `laser_z` | 向上为正 | 雷达离地高度 |
| `camera_yaw` | 逆时针为正，单位弧度 | 摄像头朝向修正 |

如果 `laser_link` 位置不对，常见现象是：

| 现象 | 可能原因 |
| --- | --- |
| 地图墙体整体偏移 | `laser_x/y` 和实际安装位置不一致 |
| 转弯时地图撕裂 | `odom -> base_link` 或 `base_link -> laser_link` 不稳定 |
| RViz 中雷达点和车体重合不合理 | 雷达静态 TF 没调 |

## 8. 雷达参数

文件：`src/ydlidar_ros2_driver/params/ydlidar.yaml`

关键参数：

| 参数 | 当前值 | 修改效果 |
| --- | --- | --- |
| `port` | `/dev/ydlidar` | 雷达串口，推荐用 udev 固定别名 |
| `frame_id` | `laser_link` | `/scan.header.frame_id`，必须和 TF 里的雷达 frame 一致 |
| `baudrate` | `115200` | X3 常用 115200，改错会无数据 |
| `sample_rate` | `3` | 雷达采样率参数，需匹配型号 |
| `scan_frequency_hz` | `10.0` | 扫描频率，越高更新越快，但数据负载更高 |
| `min_angle_deg` / `max_angle_deg` | `-180` / `180` | 使用的扫描角度范围 |
| `min_range_m` | `0.08` | 小于该距离的数据丢弃 |
| `max_range_m` | `16.0` | 大于该距离的数据丢弃 |
| `reversion` | `false` | 扫描方向反了时可以尝试切换 |
| `inverted` | `false` | 雷达倒装或方向镜像时可以尝试切换 |
| `single_channel` | `true` | X3 常用单通道模式 |

带注释示例：

```yaml
ydlidar_ros2_driver_node:
  ros__parameters:
    port: "/dev/ydlidar"      # 雷达串口
    frame_id: "laser_link"    # 必须能通过 TF 连到 base_link
    baudrate: 115200          # 和雷达型号匹配
    scan_frequency_hz: 10.0   # 扫描更新频率
    reversion: false          # 左右镜像/方向反时尝试改 true
    inverted: false           # 倒装或角度反时尝试改 true
```

调试命令：

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

## 9. GMapping 参数

文件：`src/slam_gmapping/params/slam_gmapping.yaml`

GMapping 的输入是：

| 输入 | 作用 |
| --- | --- |
| `/scan` | 激光雷达数据 |
| `odom -> base_link` | 小车短时间运动估计 |
| `base_link -> laser_link` | 雷达安装位置 |

GMapping 的输出是：

| 输出 | 作用 |
| --- | --- |
| `/map` | 实时地图 |
| `map -> odom` | 全局定位修正 |

关键参数：

| 参数 | 当前值 | 改大效果 | 改小效果 |
| --- | --- | --- | --- |
| `delta` | `0.05` | 地图分辨率变粗，计算更轻 | 地图更细，计算更重 |
| `particles` | `30` | 更稳但更耗 CPU | 更省 CPU 但容易建图不稳 |
| `linearUpdate` | `1.0` | 走更远才更新一次，省 CPU，地图更新慢 | 更频繁更新，地图更实时 |
| `angularUpdate` | `0.5` | 转更多角度才更新，省 CPU | 转动时地图更新更频繁 |
| `temporalUpdate` | `1.0` | 时间更新间隔更长 | 地图时间更新更快 |
| `map_update_interval` | `0.1` | `/map` 发布更慢 | `/map` 发布更快 |
| `maxUrange` | `4.0` | 使用更远雷达点建图 | 只用近处点，远墙可能不进图 |
| `maxRange` | `6.0` | 允许更远最大量程 | 丢弃远距离点 |
| `minimum_score` | `0.0` | 匹配要求更严格时可提高 | 更宽松，可能接受差匹配 |
| `srr/srt/str/stt` | `0.1/0.2/0.1/0.2` | 里程计噪声模型更大，算法更不信任里程计 | 更信任里程计 |
| `xmin/xmax/ymin/ymax` | `-10/10/-10/10` | 地图初始范围更大 | 地图范围更小 |

带注释示例：

```yaml
/slam_gmapping:
  ros__parameters:
    base_frame: base_link
    odom_frame: odom
    map_frame: map
    delta: 0.05              # 地图每个栅格 5cm
    particles: 30            # 粒子数，越大越稳也越耗 CPU
    linearUpdate: 1.0        # 直线移动 1m 后强制更新
    angularUpdate: 0.5       # 旋转 0.5rad 后强制更新
    map_update_interval: 0.1 # /map 发布间隔
    maxUrange: 4.0           # 真正用于建图的有效量程
```

如果你想让地图更实时，可以先试：

```yaml
linearUpdate: 0.3
angularUpdate: 0.2
temporalUpdate: 0.5
```

如果 CPU 占用变高或地图抖动，再逐步调回去。

## 10. Nav2 总览

文件：`src/my_pkg/config/nav2_params.yaml`

Nav2 不是一个节点，而是一组 lifecycle 节点。你现在看到的这些节点都属于 Nav2：

| 节点 | 作用 |
| --- | --- |
| `planner_server` | 根据地图规划全局路径 `/plan` |
| `controller_server` | 沿着路径生成速度 `/cmd_vel` |
| `bt_navigator` | 行为树总调度，接收 RViz 的目标点 |
| `behavior_server` | 倒车、原地旋转、等待等恢复行为 |
| `smoother_server` | 平滑路径 |
| `velocity_smoother` | 平滑速度命令，限制加速度和死区 |
| `local_costmap` | 车附近的局部障碍代价地图 |
| `global_costmap` | 全局地图上的障碍和膨胀层 |

检查 lifecycle 状态：

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /behavior_server
```

正常应该是：

```text
active [3]
```

如果是 `unconfigured [1]`，通常说明 Nav2 没有被 lifecycle manager 自动配置，或者启动参数/TF/地图还没准备好。

## 11. Nav2 速度参数

当前你已经重点调过速度。单位要特别注意：

| ROS2 单位 | 含义 |
| --- | --- |
| `linear.x = 0.10` | `0.10 m/s`，也就是 `10 cm/s` |
| `angular.z = 1.2` | `1.2 rad/s`，角速度 |

### 控制器速度

位置：`controller_server -> FollowPath`

| 参数 | 当前值 | 修改效果 |
| --- | --- | --- |
| `min_vel_x` | `0.10` | DWB 采样的最小前进速度 |
| `max_vel_x` | `0.30` | 最大前进速度 |
| `min_speed_xy` | `0.10` | 平面移动速度下限 |
| `max_speed_xy` | `0.30` | 平面移动速度上限 |
| `min_vel_theta` | `-1.2` | 最大右转角速度下限方向 |
| `max_vel_theta` | `1.2` | 最大左转角速度 |
| `min_speed_theta` | `0.50` | 原地转动时最小角速度 |
| `trans_stopped_velocity` | `0.10` | 小于该线速度认为已经停止 |

你看到 `/cmd_vel` 有 `linear.x=-0.05` 时，后面把 `trans_stopped_velocity` 和 `deadband_velocity` 调到了 `0.10`。这表示小于 `0.10 m/s` 的小速度会被视为停止/死区。

注意：这不是“把 0.05 强行抬到 0.10”，而是“低于 0.10 就当作 0”。如果要强制所有非零速度至少正负 `0.10`，需要额外写一个 `/cmd_vel` 后处理限幅节点。

### 速度平滑器

位置：`velocity_smoother`

| 参数 | 当前值 | 修改效果 |
| --- | --- | --- |
| `max_velocity` | `[0.20, 0.0, 1.2]` | 平滑后最大速度，分别是 x、y、theta |
| `min_velocity` | `[-0.10, 0.0, -1.2]` | 平滑后最小速度 |
| `max_accel` | `[0.6, 0.0, 2.0]` | 最大加速度 |
| `max_decel` | `[-0.6, 0.0, -2.0]` | 最大减速度 |
| `deadband_velocity` | `[0.10, 0.0, 0.0]` | 小于该线速度的命令压成 0 |
| `smoothing_frequency` | `20.0` | 平滑器输出频率 |

如果小车启动太猛，降低 `max_accel`。

如果小车刹停太慢，增大减速度绝对值，例如 `max_decel: [-0.8, 0.0, -2.5]`。

如果小车低速不动，不要盲目降低 `min_vel_x`，因为电机有静摩擦。更合理的做法是先确认底盘能稳定执行 `0.10 m/s`。

### 手动测试速度

```bash
# 前进 10cm/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.10}, angular: {z: 0.0}}"

# 原地左转
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}"

# 停车
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

## 12. Nav2 路径规划参数

位置：`planner_server`

| 参数 | 当前值 | 修改效果 |
| --- | --- | --- |
| `expected_planner_frequency` | `5.0` | 期望全局规划频率 |
| `use_astar` | `false` | `false` 使用 Dijkstra 风格；`true` 使用 A* |
| `allow_unknown` | `true` | 是否允许路径穿过未知区域 |
| `tolerance` | `0.5` | 目标点附近可接受误差 |

如果实时建图时地图未知区域很多，`allow_unknown: true` 更容易规划出路；如果你只想走已知区域，可以改成 `false`，但探索阶段可能经常规划失败。

RViz 里显示路径：

| Display 类型 | Topic |
| --- | --- |
| `Path` | `/plan` |
| `Path` | `/plan_smoothed` |
| `Path` | `/local_plan` |

## 13. Nav2 代价地图参数

代价地图就是 Nav2 对障碍物危险程度的理解。RViz 里你看到的深浅灰色膨胀区域，大多来自 costmap 的 inflation layer。

### local_costmap

局部代价地图跟着小车走，用于避开近处障碍。

| 参数 | 当前值 | 修改效果 |
| --- | --- | --- |
| `global_frame` | `odom` | 局部地图跟随里程计坐标，短时间稳定 |
| `rolling_window` | `true` | 地图窗口跟着车移动 |
| `width` / `height` | `3` / `3` | 局部地图 3m x 3m |
| `resolution` | `0.05` | 每格 5cm |
| `robot_radius` | `0.16` | 小车半径，影响障碍避让距离 |
| `inflation_radius` | `0.18` | 障碍膨胀半径 |
| `cost_scaling_factor` | `3.0` | 膨胀代价衰减速度 |

如果小车离障碍太近，增大：

```yaml
robot_radius: 0.18
inflation_radius: 0.35
```

如果通道明明能过但导航不敢走，适当减小：

```yaml
robot_radius: 0.14
inflation_radius: 0.20
```

### global_costmap

全局代价地图用于全局路径规划。

| 参数 | 当前值 | 修改效果 |
| --- | --- | --- |
| `global_frame` | `map` | 全局地图坐标 |
| `track_unknown_space` | `true` | 保留未知区域 |
| `static_layer` | 开启 | 订阅 `/map` |
| `obstacle_layer` | 开启 | 用 `/scan` 实时标障碍 |
| `inflation_radius` | `0.22` | 全局障碍膨胀半径 |

你在 RViz 里打开 `global_costmap` 后看到大片灰色，是正常现象。它不是原始地图，而是给规划器看的“危险区域”。灰色越大，说明 Nav2 认为离障碍越近越危险。

带注释示例：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: "map"
      robot_base_frame: "base_link"
      robot_radius: 0.16        # 车体安全半径
      resolution: 0.05          # 每格 5cm
      track_unknown_space: true # 保留未知区域
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      inflation_layer:
        inflation_radius: 0.22  # 障碍物向外扩张 22cm
        cost_scaling_factor: 3.0
```

## 14. Nav2 controller critics

位置：`controller_server -> FollowPath -> critics`

DWB 控制器会采样很多条候选轨迹，然后用 critics 打分。分数越低越好。

| critic | 作用 |
| --- | --- |
| `RotateToGoal` | 到目标附近后调整朝向 |
| `Oscillation` | 避免前后左右来回抖动 |
| `BaseObstacle` | 避障 |
| `GoalAlign` | 让车头朝向目标方向 |
| `PathAlign` | 让车头沿着路径方向 |
| `PathDist` | 惩罚偏离全局路径 |
| `GoalDist` | 惩罚离目标远 |

权重参数：

| 参数 | 当前值 | 改大后 |
| --- | --- | --- |
| `PathAlign.scale` | `32.0` | 更努力让车头对齐路径 |
| `PathDist.scale` | `32.0` | 更不愿偏离路径 |
| `GoalAlign.scale` | `24.0` | 更重视朝向目标 |
| `GoalDist.scale` | `24.0` | 更重视接近目标 |
| `BaseObstacle.scale` | `0.02` | 更害怕障碍，但过大可能不敢走 |

调参建议：先不要大幅改 critics。你现在的小车更需要先保证 `/odom`、TF、雷达方向、速度闭环稳定。

## 15. 键盘控制节点

文件：`src/my_pkg/my_pkg/keyboard_control/keyboard_control_node.py`

默认参数：

| 参数 | 当前值 | 修改效果 |
| --- | --- | --- |
| `linear_speed_m_s` | `0.10` | W/S 前进后退速度 |
| `angular_speed_rad_s` | `0.80` | A/D 原地转动速度 |
| `publish_rate_hz` | `10.0` | `/cmd_vel` 发布频率 |
| `key_timeout_sec` | `0.30` | 超过该时间没按键就自动停车 |

单独启动：

```bash
ros2 run my_pkg keyboard_control_node
```

或跟 bringup 一起启动：

```bash
ros2 launch my_pkg bringup.launch.py use_keyboard_control:=true
```

## 16. URDF 在这个项目中的作用

当前 ROS2 使用的新包是 `xc_urdf`：

```text
src/xc_urdf/urdf/xc_urdf.urdf
src/xc_urdf/meshes/*.STL
src/xc_urdf/launch/display.launch.py
```

URDF 的好处：

| 好处 | 说明 |
| --- | --- |
| RViz 显示真实车体 | 不只是看到一个 TF 坐标轴，而是能看到车壳、轮子、雷达位置 |
| 统一传感器安装位置 | 雷达、IMU、相机的 TF 可以从 URDF 统一发布 |
| 方便检查坐标系 | 车头方向、雷达朝向、轮子位置更直观 |
| 后续接仿真 | Gazebo、robot_state_publisher 都依赖 URDF |
| 为 Nav2 footprint 提供参考 | 可以根据模型尺寸设置 `robot_radius` 或 footprint |

当前 `my_pkg/bringup.launch.py` 已经可以通过 `use_urdf:=true` 启动 `xc_urdf` 中的 `robot_state_publisher`。这个 URDF 主要用于 RViz2 显示车体模型；雷达和 IMU 的 TF 仍由 `tf_tree_node` 发布。

你的使用方式是 Muse Pi 跑机器人节点，VM 跑 RViz2。Muse Pi 不需要打开 RViz2，只需要发布 URDF、TF、地图、雷达和导航话题。

Muse Pi 上启动：

```bash
ros2 launch my_pkg bringup.launch.py use_lidar:=true use_urdf:=true use_urdf_rviz:=false
```

VM 上启动 RViz2：

```bash
rviz2
```

VM 上也要有同一份 `xc_urdf` 包并执行过 `source install/setup.bash`。原因是 URDF 里 STL 路径使用 `package://xc_urdf/meshes/...`，RViz2 显示模型时需要在 VM 本地找到这些 mesh 文件。

RViz2 中添加：

```text
Fixed Frame: map
Add -> RobotModel
Add -> TF
Add -> By topic -> /scan -> LaserScan
Add -> By topic -> /map -> Map
Add -> Path -> /plan
```

后续如果要继续完善 URDF，建议方向是：

1. 把 URDF 的 `base_link` 和当前项目 frame 保持一致。
2. 用 `robot_state_publisher` 发布 URDF 中的静态关节 TF。
3. 避免 `tf_tree_node` 和 URDF 同时发布同一条 TF，否则会冲突。
4. Nav2 仍继续使用 `base_link` 和 `/scan`，只要 TF 树连续即可。

## 17. 常见问题和调参方向

### `/cmd_vel` 还有小于 0.1 的速度

检查：

```bash
ros2 topic echo /cmd_vel
```

相关参数：

```yaml
min_vel_x: 0.10
min_speed_xy: 0.10
trans_stopped_velocity: 0.10
deadband_velocity: [0.10, 0.0, 0.0]
```

含义：小于 `0.10 m/s` 的速度会被当作停止或压成 0。它不会自动把 `0.05` 抬成 `0.10`。

### 小车转不动

先手动测试：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}"
```

相关参数：

```yaml
max_vel_theta: 1.2
min_speed_theta: 0.50
max_rotational_vel: 1.2
min_rotational_vel: 0.5
```

如果 ROS 有角速度但轮子不动，可能是：

1. 电机静摩擦太大。
2. ESP32 速度闭环最小速度太低。
3. `wheel_base_m` 不准确。
4. 左右轮方向或编码器方向仍有问题。

### RViz 里小车方向反了

当前修正参数：

```bash
ros2 launch my_pkg bringup.launch.py imu_yaw_sign:=-1.0
```

如果左转时 RViz 仍右转，试：

```bash
ros2 launch my_pkg bringup.launch.py imu_yaw_sign:=1.0
```

### 地图旋转或撕裂

重点检查：

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_link
ros2 topic hz /scan
ros2 topic hz /odom
```

常见原因：

| 现象 | 可能原因 |
| --- | --- |
| 地图跟着车一起转 | `map -> odom` 没出来 |
| 墙体重影 | 雷达 TF 或 odom yaw 不准 |
| 雷达点方向反 | `inverted` / `reversion` 参数不对 |
| 建图延迟大 | `/scan` 或 `/odom` 频率低，CPU 忙 |

### local_costmap 和 global_costmap 看起来很大

这是正常的。costmap 不是原始地图，它会把障碍物按 `inflation_radius` 膨胀，给导航留安全距离。

如果灰色膨胀区太大：

```yaml
inflation_radius: 0.20
robot_radius: 0.14
```

如果小车贴障碍太近：

```yaml
inflation_radius: 0.35
robot_radius: 0.18
```

## 18. 修改参数后的生效方式

Launch 文件、Python 节点、YAML 配置修改后，一般需要重新构建并 source：

```bash
cd ~/ros2_car
colcon build --symlink-install --packages-select my_pkg
source install/setup.bash
```

如果改的是 `slam_gmapping`：

```bash
colcon build --symlink-install --packages-select slam_gmapping
source install/setup.bash
```

如果改的是 `ydlidar_ros2_driver`：

```bash
colcon build --symlink-install --packages-select ydlidar_ros2_driver
source install/setup.bash
```

如果只是改已安装包 share 目录中的 YAML，有时重启 launch 就生效；但为了避免拿到旧文件，建议每次都从工作区构建后再测。

## 19. 学习顺序建议

建议按这个顺序学习和验证：

1. 只启动底盘，确认 `/cmd_vel -> ESP32 -> /odom`。
2. 加雷达，确认 `/scan` 和 `base_link -> laser_link`。
3. 启动 GMapping，确认 `/map` 和 `map -> odom`。
4. 在 RViz 里看 `/map`、`/scan`、TF。
5. 启动 Nav2，先看 `/plan`，再看 `/cmd_vel`。
6. 最后再调速度、costmap、URDF 显示。

每次只改一个参数，然后记录现象。ROS2 调车最怕一次改太多，因为现象会互相覆盖。
