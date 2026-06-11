# ROS-CAR Bringup Steps

本文记录 Muse Pi 上当前已经跑通的小车 ROS2 启动、控制和反馈检查步骤。

## 1. 进入工作区

```bash
cd ~/ros_car
```

## 2. 确认 Python 包标记文件

如果运行节点时报错：

```text
ModuleNotFoundError: No module named 'my_pkg'
```

执行：

```bash
printf '"""ROS-CAR Python package."""\n' > src/my_pkg/my_pkg/__init__.py
```

这个文件用于让 `setuptools.find_packages()` 正确安装 `my_pkg` Python 包。

## 3. 编译并加载环境

```bash
colcon build --packages-select my_pkg
source install/setup.bash
```

每次打开新终端后，都需要重新执行：

```bash
cd ~/ros_car
source install/setup.bash
```

## 4. 启动 ESP32 桥接节点

```bash
ros2 run my_pkg esp32_bridge_node --ros-args -p port:=/dev/ttyUSB0 -p enable_print:=true
```

如果串口不是 `/dev/ttyUSB0`，先查看实际设备：

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

例如实际是 `/dev/ttyACM0`，则改为：

```bash
ros2 run my_pkg esp32_bridge_node --ros-args -p port:=/dev/ttyACM0 -p enable_print:=true
```

## 5. 查看话题

新开一个终端，加载环境后执行：

```bash
ros2 topic list
```

当前已确认能看到：

```text
/cmd_vel
/esp32/status
/imu/data
/imu/rawdata
/odom
/parameter_events
/rosout
```

## 6. 每 100 ms 发送一次速度

前进：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.0}}"
```

停止：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

后退：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.15}, angular: {z: 0.0}}"
```

原地左转：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.8}}"
```

原地右转：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.8}}"
```

## 7. 查看里程计反馈

```bash
ros2 topic echo /odom
```

发送速度时，重点观察：

```text
pose.pose.position.x
pose.pose.position.y
twist.twist.linear.x
```

当前已经确认 `/odom` 可以正常发布，并且小车运动时位置会变化。

## 8. 查看 IMU 反馈

查看原始 IMU：

```bash
ros2 topic echo /imu/rawdata
```

查看带姿态的 IMU：

```bash
ros2 topic echo /imu/data
```

只看一条：

```bash
ros2 topic echo --once /imu/data
```

重点观察：

```text
orientation
angular_velocity
linear_acceleration
```

## 9. 关于 /esp32/status

如果执行：

```bash
ros2 topic echo /esp32/status
```

出现：

```text
WARNING: topic [/esp32/status] does not appear to be published yet
Could not determine the type for the passed topic
```

先检查节点是否还在运行：

```bash
ros2 node list
```

如果 `/odom` 正常发布，说明桥接节点和串口反馈链路已经在工作，`/esp32/status` 可以后续再优化为周期发布或 transient local QoS。

## 10. 下一步：键盘遥控

如果系统已经安装键盘遥控包，可以运行：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

如果提示找不到包，安装：

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
```

键盘遥控成功后，就不需要反复手动输入 `ros2 topic pub` 命令。
