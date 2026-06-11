# my_pkg 节点功能与运行说明

本文档用于检查当前 `my_pkg` 功能包里的主要节点：每个节点做什么、怎么运行、运行后发布/订阅哪些话题，以及这些话题在小车系统里干什么。

当前已有节点：

- `esp32_bridge_node`
- `tf_tree_node`
- `keyboard_control_node`

当前已有一键启动文件：

- `bringup.launch.py`

## 总体数据流

```text
keyboard_control_node
  -> /cmd_vel
  -> esp32_bridge_node
  -> ESP32
  -> 电机

ESP32
  -> 编码器和 IMU 串口数据
  -> esp32_bridge_node
  -> /odom /imu/rawdata /imu/data /esp32/status

/odom
  -> tf_tree_node
  -> /tf: odom -> base_link

tf_tree_node
  -> /tf_static: base_link -> base_footprint / imu_link / laser_link / camera_link
```

## 1. esp32_bridge_node

### 功能

`esp32_bridge_node` 是 Muse Pi 和 ESP32 下位机之间的桥接节点。

它主要做四件事：

- 订阅 `/cmd_vel`。
- 把 `/cmd_vel` 中的小车线速度和角速度转换成左右轮目标速度。
- 通过串口把左右轮目标速度发送给 ESP32。
- 接收 ESP32 回传的编码器和 IMU 数据，并发布 ROS2 话题。

### 运行方式

单独运行：

```bash
ros2 run my_pkg esp32_bridge_node --ros-args -p port:=/dev/ttyUSB0
```

如果实际串口是 `/dev/ttyACM0`：

```bash
ros2 run my_pkg esp32_bridge_node --ros-args -p port:=/dev/ttyACM0
```

打开普通调试打印：

```bash
ros2 run my_pkg esp32_bridge_node --ros-args -p port:=/dev/ttyUSB0 -p enable_print:=true
```

通过 launch 运行：

```bash
ros2 launch my_pkg bringup.launch.py port:=/dev/ttyUSB0
```

### 参数

| 参数名 | 默认值 | 作用 |
| --- | --- | --- |
| `port` | `/dev/ttyUSB0` | ESP32 串口设备名 |
| `baudrate` | `115200` | 串口波特率 |
| `wheel_base_m` | `0.18` | 左右驱动轮中心距，单位 m |
| `enable_print` | `false` | 是否打印普通串口/速度调试信息 |

### 订阅话题

| 话题 | 类型 | 来源 | 作用 |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `keyboard_control_node`、后续 Nav2 或其它控制节点 | 小车运动速度命令 |

`/cmd_vel` 中当前主要使用：

- `linear.x`：前进/后退速度，单位 m/s。
- `angular.z`：左转/右转角速度，单位 rad/s。

节点收到 `/cmd_vel` 后，会根据差速模型换算左右轮速度，并发送给 ESP32。

### 发布话题

| 话题 | 类型 | 使用者 | 作用 |
| --- | --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | `tf_tree_node`、后续 SLAM/Nav2 | 小车里程计位姿和速度 |
| `/imu/rawdata` | `sensor_msgs/msg/Imu` | 调试、后续滤波节点 | MPU6050 原始加速度和角速度 |
| `/imu/data` | `sensor_msgs/msg/Imu` | 后续 EKF、姿态显示 | 带 ESP32 解算姿态的 IMU 数据 |
| `/esp32/status` | `std_msgs/msg/String` | 调试、后续状态管理节点 | ESP32 串口连接状态 |

### 运行后的预期结果

运行成功后，终端会看到类似：

```text
esp32_bridge_node initialized successfully
opened serial port /dev/ttyUSB0 at 115200
```

可以检查话题：

```bash
ros2 topic list
ros2 topic echo /esp32/status
ros2 topic echo /odom
ros2 topic echo /imu/data
```

如果 ESP32 正常回传编码器和 IMU 数据：

- `/odom` 会持续更新。
- `/imu/rawdata` 会持续更新。
- `/imu/data` 会持续更新。
- `/esp32/status` 会发布 `initialized`、`serial_connected` 等状态。

如果串口断开：

- 节点会发布 `serial_disconnected`。
- 节点会定时尝试重连。

## 2. tf_tree_node

### 功能

`tf_tree_node` 负责发布小车当前阶段需要的 TF 坐标树。

它主要做两件事：

- 订阅 `/odom`，把里程计位姿转换成动态 TF：`odom -> base_link`。
- 发布固定安装关系的静态 TF：`base_link -> base_footprint`、`base_link -> imu_link`、`base_link -> laser_link`、`base_link -> camera_link`。

注意：当前还没有 URDF 文件，所以静态 TF 暂时由这个节点发布。后续如果加入 URDF，静态 TF 必须遵循 URDF 的 `link/joint` 内容，避免和 `robot_state_publisher` 重复发布同一段 TF。

### 运行方式

单独运行：

```bash
ros2 run my_pkg tf_tree_node
```

通过 launch 运行：

```bash
ros2 launch my_pkg bringup.launch.py
```

`bringup.launch.py` 会自动加载：

```text
config/tf_params.yaml
```

### 参数文件

TF 坐标偏移统一写在：

```text
ros_car/src/my_pkg/config/tf_params.yaml
```

当前参数包括：

| 参数名 | 作用 |
| --- | --- |
| `odom_topic` | 订阅哪个里程计话题，默认 `/odom` |
| `odom_frame` | 里程计父坐标系，默认 `odom` |
| `base_frame` | 车体主坐标系，默认 `base_link` |
| `base_footprint_frame` | 地面投影坐标系，默认 `base_footprint` |
| `imu_frame` | IMU 坐标系，默认 `imu_link` |
| `laser_frame` | 雷达坐标系，默认 `laser_link` |
| `camera_frame` | 摄像头坐标系，默认 `camera_link` |
| `base_footprint_x/y/z` | `base_footprint` 相对 `base_link` 的偏移 |
| `imu_x/y/z` | IMU 相对 `base_link` 的偏移 |
| `laser_x/y/z` | 雷达相对 `base_link` 的偏移 |
| `camera_x/y/z` | 摄像头相对 `base_link` 的偏移 |
| `camera_yaw` | 摄像头相对 `base_link` 的 yaw 角，单位 rad |

### 订阅话题

| 话题 | 类型 | 来源 | 作用 |
| --- | --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | `esp32_bridge_node` | 提供小车在 `odom` 坐标系下的位置和姿态 |

### 发布内容

| 输出 | 内容 | 使用者 | 作用 |
| --- | --- | --- | --- |
| `/tf` | `odom -> base_link` | RViz2、SLAM、Nav2 | 表示小车在里程计坐标系中的实时位置 |
| `/tf_static` | `base_link -> base_footprint` | RViz2、SLAM、Nav2 | 表示底盘地面投影位置 |
| `/tf_static` | `base_link -> imu_link` | RViz2、后续 EKF | 表示 IMU 安装位置 |
| `/tf_static` | `base_link -> laser_link` | RViz2、后续雷达/SLAM | 表示雷达安装位置 |
| `/tf_static` | `base_link -> camera_link` | RViz2、后续摄像头节点 | 表示摄像头安装位置 |

### 运行后的预期结果

如果 `/odom` 正常发布，`tf_tree_node` 会发布动态 TF：

```text
odom -> base_link
```

可以检查：

```bash
ros2 run tf2_tools view_frames
```

或打开 RViz2：

```bash
rviz2
```

RViz2 中建议：

- `Fixed Frame` 设置为 `odom`。
- 添加 `TF` 显示。
- 添加 `Odometry` 显示，话题选择 `/odom`。

小车移动时，RViz2 中的 `base_link` 和 `/odom` 箭头应跟着移动。

## 3. keyboard_control_node

### 功能

`keyboard_control_node` 是基础键盘遥控节点。

它只负责一件事：

```text
键盘 W/A/S/D
  -> /cmd_vel
```

它不直接控制串口，也不直接控制 ESP32。真正发送串口控制帧的是 `esp32_bridge_node`。

### 运行方式

推荐单独开一个终端运行：

```bash
ros2 run my_pkg keyboard_control_node
```

原因：键盘控制需要读取当前终端输入，单独运行比放在多节点 launch 里更稳定。

也可以通过 launch 可选启动：

```bash
ros2 launch my_pkg bringup.launch.py use_keyboard_control:=true
```

### 按键

| 按键 | 效果 | 发布到 `/cmd_vel` 的含义 |
| --- | --- | --- |
| `W` | 前进 | `linear.x > 0` |
| `S` | 后退 | `linear.x < 0` |
| `A` | 左转 | `angular.z > 0` |
| `D` | 右转 | `angular.z < 0` |
| `Space` | 停止 | `linear.x = 0`，`angular.z = 0` |
| `X` | 停止 | `linear.x = 0`，`angular.z = 0` |
| `Ctrl-C` | 退出节点 | 退出前发布一次停止命令 |

### 参数

| 参数名 | 默认值 | 作用 |
| --- | --- | --- |
| `linear_speed_m_s` | `0.10` | 前进/后退速度，单位 m/s |
| `angular_speed_rad_s` | `0.80` | 左转/右转角速度，单位 rad/s |
| `publish_rate_hz` | `10.0` | `/cmd_vel` 发布频率 |
| `key_timeout_sec` | `0.30` | 超过多久没有有效按键后自动发停止 |

示例：降低速度运行：

```bash
ros2 run my_pkg keyboard_control_node --ros-args -p linear_speed_m_s:=0.05 -p angular_speed_rad_s:=0.4
```

### 发布话题

| 话题 | 类型 | 使用者 | 作用 |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `esp32_bridge_node` | 小车速度控制命令 |

### 订阅话题

无。

### 运行后的预期结果

运行成功后，终端会看到：

```text
keyboard_control_node initialized: W/S forward/backward, A/D turn, Space/X stop
```

可以检查 `/cmd_vel`：

```bash
ros2 topic echo /cmd_vel
```

按下 `W` 时，`linear.x` 应为正数。

按下 `S` 时，`linear.x` 应为负数。

按下 `A` 时，`angular.z` 应为正数。

按下 `D` 时，`angular.z` 应为负数。

松开键超过 `0.3` 秒后，节点会自动发布 0 速度。ESP32 下位机也有 500 ms 超时停车，两个机制共同保证安全。

## 4. bringup.launch.py

### 功能

`bringup.launch.py` 是当前 `my_pkg` 的一键启动文件。

默认启动：

- `esp32_bridge_node`
- `tf_tree_node`

可选启动：

- `keyboard_control_node`

### 运行方式

默认一键启动：

```bash
ros2 launch my_pkg bringup.launch.py port:=/dev/ttyUSB0
```

打开 ESP32 普通调试打印：

```bash
ros2 launch my_pkg bringup.launch.py port:=/dev/ttyUSB0 enable_print:=true
```

同时启动键盘控制：

```bash
ros2 launch my_pkg bringup.launch.py port:=/dev/ttyUSB0 use_keyboard_control:=true
```

实车测试时，更推荐：

终端 1：

```bash
ros2 launch my_pkg bringup.launch.py port:=/dev/ttyUSB0
```

终端 2：

```bash
ros2 run my_pkg keyboard_control_node
```

### launch 参数

| 参数名 | 默认值 | 作用 |
| --- | --- | --- |
| `port` | `/dev/ttyUSB0` | ESP32 串口设备名 |
| `baudrate` | `115200` | ESP32 串口波特率 |
| `wheel_base_m` | `0.18` | 左右驱动轮中心距 |
| `enable_print` | `false` | 是否打开 ESP32 bridge 普通调试打印 |
| `use_keyboard_control` | `false` | 是否随 launch 启动键盘控制节点 |

### 运行后的预期结果

默认启动后：

- `esp32_bridge_node` 负责串口和底盘数据。
- `tf_tree_node` 负责 TF 坐标树。
- 如果 ESP32 正常回传编码器数据，会有 `/odom`。
- 如果 `/odom` 正常，TF 中会有 `odom -> base_link`。

可以检查：

```bash
ros2 node list
ros2 topic list
ros2 topic echo /esp32/status
ros2 topic echo /odom
```

## 5. 当前最小实车测试流程

### 1. 构建

```bash
cd ~/ros_car
colcon build --packages-select my_pkg
source install/setup.bash
```

### 2. 启动底盘和 TF

```bash
ros2 launch my_pkg bringup.launch.py port:=/dev/ttyUSB0
```

### 3. 启动键盘控制

另开一个终端：

```bash
source ~/ros_car/install/setup.bash
ros2 run my_pkg keyboard_control_node
```

### 4. 观察话题

另开一个终端：

```bash
source ~/ros_car/install/setup.bash
ros2 topic echo /cmd_vel
ros2 topic echo /odom
```

### 5. 打开 RViz2

```bash
rviz2
```

建议设置：

- `Fixed Frame`: `odom`
- Add `TF`
- Add `Odometry`，topic 选择 `/odom`

预期结果：

- 按 `W/A/S/D` 时，真实小车移动。
- `/cmd_vel` 有对应速度命令。
- ESP32 回传编码器后，`/odom` 更新。
- `tf_tree_node` 发布 `odom -> base_link`。
- RViz2 中的小车坐标或里程计箭头跟着移动。

## 6. 话题关系总表

| 话题 | 类型 | 发布者 | 订阅者 | 用途 |
| --- | --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `keyboard_control_node` | `esp32_bridge_node` | 手动控制小车速度 |
| `/odom` | `nav_msgs/msg/Odometry` | `esp32_bridge_node` | `tf_tree_node`、RViz2、后续 SLAM/Nav2 | 小车里程计 |
| `/imu/rawdata` | `sensor_msgs/msg/Imu` | `esp32_bridge_node` | 调试、后续滤波 | 原始 IMU 数据 |
| `/imu/data` | `sensor_msgs/msg/Imu` | `esp32_bridge_node` | RViz2、后续 EKF/Nav2 | 带姿态的 IMU 数据 |
| `/esp32/status` | `std_msgs/msg/String` | `esp32_bridge_node` | 调试、后续 robot_manager | ESP32 串口连接状态 |
| `/tf` | `tf2_msgs/msg/TFMessage` | `tf_tree_node` | RViz2、SLAM、Nav2 | 动态坐标变换 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `tf_tree_node` | RViz2、SLAM、Nav2 | 静态坐标变换 |

## 7. 后续注意

- 后续如果加入 URDF，静态 TF 要以 URDF 中的 `link/joint` 为准，避免 `tf_tree_node` 和 `robot_state_publisher` 重复发布同一段静态 TF。
- 后续如果加入 Nav2，Nav2 也会发布或消费 `/cmd_vel`，那时需要考虑控制权管理，避免键盘控制和 Nav2 同时抢 `/cmd_vel`。
- 后续如果要做更完整的遥控控制，可以增加 `cmd_vel_mux` 或 `cmd_vel_controller_node`，负责急停、速度限幅、输入源切换和速度平滑。
