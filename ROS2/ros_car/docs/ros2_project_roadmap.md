# ROS2 Project Roadmap

本文档用于指导 ROS-CAR 项目后续开发。当前项目先按 16 个 ROS2 节点规划，目标是保持架构规范，同时避免早期功能膨胀。

## 1. Overall Goal

项目最终形态：

- Muse Pi 运行底盘、传感器、SLAM、Nav2、总控和显示相关节点。
- ESP32 负责电机、编码器、IMU、电池等底层硬件。
- 电脑可运行 PyBullet 五自由度机械臂仿真节点。
- Muse Pi 和电脑处于同一个 ROS2 网络，通过话题、服务和 Action 协同。

真正的开发主线：

```text
ROS2 基础
  -> /cmd_vel 控 ESP32
  -> /odom /imu
  -> TF
  -> 雷达 /scan
  -> SLAM 建图
  -> Nav2 导航
  -> 摄像头
  -> PyBullet 机械臂
  -> robot_manager 总控
  -> 屏幕 UI
```

最先做、最重要的节点只有一个：

```text
esp32_bridge_node
```

它跑通以后，项目才真正进入可扩展阶段。

## 2. Node List

### 2.1 Hardware Driver Nodes

#### esp32_bridge_node

作用：负责 Muse Pi 和 ESP32 通信。

订阅：

- `/cmd_vel`

发布：

- `/odom`
- `/imu/data`
- `/esp32/status`

#### lidar_node

作用：激光雷达驱动。

发布：

- `/scan`

#### camera_node

作用：摄像头驱动。

发布：

- `/image_raw`
- `/camera_info`

#### display_node

作用：屏幕显示驱动。

订阅：

- `/robot_state`
- `/esp32/status`
- `/arm/status`

### 2.2 Robot Model And TF Nodes

#### robot_state_publisher

作用：根据 URDF 发布机器人静态和动态 TF。

订阅：

- `/joint_states`

发布：

- `/tf`
- `/tf_static`

#### static_transform_publisher

作用：发布固定坐标变换。

典型固定变换：

- `base_link -> laser_link`
- `base_link -> camera_link`
- `base_link -> imu_link`

### 2.3 Localization And Fusion Node

#### ekf_filter_node

来源：`robot_localization`

作用：融合编码器里程计和 IMU。

订阅：

- `/odom`
- `/imu/data`

发布：

- `/odometry/filtered`

### 2.4 Mapping And Navigation Nodes

#### slam_toolbox_node

作用：激光雷达建图。

订阅：

- `/scan`
- `/odom` 或 `/odometry/filtered`
- `/tf`

发布：

- `/map`

#### Nav2 Node Group

Nav2 通常不是一个单独节点，而是一组节点。

常见节点：

- `map_server`
- `amcl`
- `planner_server`
- `controller_server`
- `bt_navigator`
- `behavior_server`
- `waypoint_follower`
- `velocity_smoother`
- `collision_monitor`
- `lifecycle_manager_navigation`

最重要的节点：

- `planner_server`：全局路径规划。
- `controller_server`：局部控制。
- `bt_navigator`：行为树导航。
- `amcl`：地图定位。
- `map_server`：加载地图。

Nav2 输入：

- `/map`
- `/scan`
- `/tf`
- `/odometry/filtered`

Nav2 输出：

- `/cmd_vel`

### 2.5 Arm Simulation Node

#### arm_sim_node

运行位置：电脑。

作用：运行 PyBullet 五自由度机械臂仿真。

订阅：

- `/arm/target_pose`
- `/arm/joint_command`
- `/arm/gripper_command`

发布：

- `/joint_states`
- `/arm/end_effector_pose`
- `/arm/status`

服务：

- `/arm/home`
- `/arm/reset`

### 2.6 Perception Node

#### object_detector_node

作用：目标检测，后期再实现。

订阅：

- `/image_raw`

发布：

- `/detected_objects`

### 2.7 Manager Node

#### robot_manager_node

作用：核心总控节点，负责状态机、任务调度、模式切换和异常处理。

订阅：

- `/esp32/status`
- `/arm/status`
- `/detected_objects`

发布：

- `/robot_state`
- `/arm/target_pose`

调用：

- Nav2 `NavigateToPose` Action
- `/arm/home` Service
- `/arm/reset` Service

注意：不要太早写 `robot_manager_node`。底盘、雷达、导航、机械臂都稳定以后，总控节点才有真实价值。

### 2.8 Teleop Node

#### teleop_keyboard_node

可替代方案：

- `joy_node`
- `teleop_twist_joy_node`

作用：手动控制小车。

发布：

- `/cmd_vel`

## 3. Recommended Node Relationship

```text
camera_node
  -> /image_raw
  -> object_detector_node
  -> /detected_objects
  -> robot_manager_node

robot_manager_node
  -> Nav2 NavigateToPose Action
  -> /arm/target_pose

Nav2
  -> /cmd_vel
  -> esp32_bridge_node
  -> ESP32

arm_sim_node
  -> /joint_states
  -> robot_state_publisher

lidar_node
  -> /scan
  -> slam_toolbox_node / Nav2

esp32_bridge_node
  -> /odom
  -> /imu/data
  -> ekf_filter_node
  -> /odometry/filtered
  -> Nav2
```

## 4. Build Order From Zero

### Step 1: Build ROS2 Basics

先做到：

- 能运行 `talker` / `listener`
- 能创建 ROS2 Python 包
- 能写一个简单节点
- 能使用 `ros2 topic pub` 和 `ros2 topic echo`

需要掌握：

```bash
ros2 node list
ros2 topic list
ros2 topic echo
ros2 topic pub
ros2 run
ros2 launch
```

### Step 2: Build esp32_bridge_node First

这是最重要的第一步。先不接雷达、不接摄像头、不接 Nav2。

目标：

- ROS2 发布 `/cmd_vel`
- ESP32 小车能动

数据流程：

```text
/cmd_vel
  -> esp32_bridge_node
  -> serial
  -> ESP32
  -> motors
```

验收命令：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

验收结果：小车前进。

### Step 3: Return Encoder And IMU Data From ESP32

目标：

- ESP32 数据回传到 Muse Pi
- ROS2 发布 `/odom` 和 `/imu/data`

验收命令：

```bash
ros2 topic echo /odom
ros2 topic echo /imu/data
```

验收结果：能看到里程计和 IMU 数据。

### Step 4: Build The TF Tree

先建立这些坐标：

- `odom`
- `base_link`
- `base_footprint`
- `laser_link`
- `camera_link`
- `imu_link`

最小 TF：

- `odom -> base_link`
- `base_link -> laser_link`
- `base_link -> imu_link`
- `base_link -> camera_link`

验收命令：

```bash
ros2 run tf2_tools view_frames
```

验收结果：能生成完整 TF 图。

### Step 5: Connect lidar_node

目标：

- 雷达发布 `/scan`
- RViz 能看到雷达数据

验收命令：

```bash
ros2 topic echo /scan
rviz2
```

### Step 6: Run SLAM Mapping

启动：

- `slam_toolbox_node`

输入：

- `/scan`
- `/odom`
- `/tf`

输出：

- `/map`

验收结果：遥控小车走一圈，RViz 里能生成地图。

### Step 7: Save The Map

目标产物：

- `map.yaml`
- `map.pgm`

完成后，小车已经具备地图。

### Step 8: Run Nav2 Navigation

启动核心节点：

- `map_server`
- `amcl`
- `planner_server`
- `controller_server`
- `bt_navigator`

输入：

- `/map`
- `/scan`
- `/tf`
- `/odometry/filtered`

输出：

- `/cmd_vel`

验收结果：在 RViz 里点目标点，小车能自动过去。

### Step 9: Connect camera_node

发布：

- `/image_raw`
- `/camera_info`

验收结果：RViz 能看到摄像头画面。

### Step 10: Connect PyBullet arm_sim_node

运行位置：电脑。

前提：

- Muse Pi 和电脑在同一个 ROS2 网络。

订阅：

- `/arm/target_pose`

发布：

- `/joint_states`
- `/arm/status`

验收结果：Muse Pi 发目标位姿，电脑 PyBullet 机械臂动起来。

### Step 11: Build robot_manager_node

最后再写总控节点。

`robot_manager_node` 负责：

- 切换模式
- 发导航目标
- 导航完成后触发机械臂
- 处理错误
- 发布机器人状态

### Step 12: Build display_node

最后做屏幕 UI。

显示内容：

- 当前状态
- 电量
- ESP32 连接状态
- 雷达状态
- 导航状态
- 机械臂状态

## 5. Phase Checklist

### Phase 1: Base Movement

- [ ] ROS2 Python 包创建成功。
- [ ] `esp32_bridge_node` 能订阅 `/cmd_vel`。
- [ ] ESP32 能接收串口控制指令。
- [ ] 小车能前进、后退、左转、右转、停止。

### Phase 2: Feedback And TF

- [ ] ESP32 能回传编码器数据。
- [ ] ESP32 能回传 IMU 数据。
- [ ] ROS2 能 echo `/odom`。
- [ ] ROS2 能 echo `/imu/data`。
- [ ] TF 坐标树完整。

### Phase 3: Mapping

- [ ] 雷达能发布 `/scan`。
- [ ] RViz 能显示雷达数据。
- [ ] `slam_toolbox_node` 能生成 `/map`。
- [ ] 地图能保存为 `map.yaml` 和 `map.pgm`。

### Phase 4: Navigation

- [ ] Nav2 能加载地图。
- [ ] AMCL 定位正常。
- [ ] RViz 目标点能触发导航。
- [ ] Nav2 能输出 `/cmd_vel`。
- [ ] 小车能自动到达目标点。

### Phase 5: Camera And Arm

- [ ] 摄像头能发布 `/image_raw`。
- [ ] RViz 能显示摄像头画面。
- [ ] 电脑端 `arm_sim_node` 能加入同一个 ROS2 网络。
- [ ] 机械臂能响应 `/arm/target_pose`。

### Phase 6: Manager And Display

- [ ] `robot_manager_node` 能发布 `/robot_state`。
- [ ] `robot_manager_node` 能调用 Nav2 Action。
- [ ] `robot_manager_node` 能调用机械臂服务。
- [ ] `display_node` 能显示机器人状态、电量和连接状态。

