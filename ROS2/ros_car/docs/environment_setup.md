# ROS2 Environment Setup Notes

本文档记录 ROS-CAR 当前阶段用到的环境安装和串口权限操作。

## 1. Muse Pi / Ubuntu

### 1.1 安装 pyserial

`esp32_bridge_node` 需要通过 pyserial 访问 ESP32 串口。

```bash
sudo apt update
sudo apt install python3-serial
```

安装后验证：

```bash
python3 -c "import serial; print(serial.__version__)"
```

### 1.2 给当前用户串口权限

把当前用户加入 `dialout` 组：

```bash
sudo usermod -aG dialout $USER
```

执行后需要重新登录 SSH，或者直接重启：

```bash
sudo reboot
```

重新登录后检查用户组：

```bash
groups
```

输出里应该能看到：

```text
dialout
```

查看串口设备权限：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

正常情况下，串口设备所属组应为 `dialout`，例如：

```text
crw-rw---- 1 root dialout ... /dev/ttyUSB0
```

### 1.3 临时串口权限方案

如果只是临时测试，也可以给具体串口设备放开权限：

```bash
sudo chmod 666 /dev/ttyUSB0
```

注意：这个方式在重启或重新插拔串口设备后可能失效，长期使用优先采用 `dialout` 组方案。

## 2. ROS2 Package Build

进入 ROS2 工作区：

```bash
cd ros_car
```

构建包：

```bash
colcon build
```

加载环境：

```bash
source install/setup.bash
```

运行 ESP32 桥接节点：

```bash
ros2 run my_pkg esp32_bridge_node --ros-args -p port:=/dev/ttyUSB0 -p enable_print:=true
```

如果实际串口不是 `/dev/ttyUSB0`，先查看设备：

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

再把 `port` 参数改成实际设备，例如：

```bash
ros2 run my_pkg esp32_bridge_node --ros-args -p port:=/dev/ttyACM0 -p enable_print:=true
```

## 3. Windows Test Script

如果在 Windows 上运行 `serial_control.py`，使用 pip 安装 pyserial：

```bash
python -m pip install pyserial
```

验证：

```bash
python -c "import serial; print(serial.__version__)"
```

## 4. Current Base Movement Test

发送一次前进指令：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

发送停止指令：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

`linear.x = 0.2` 表示前进 `0.2 m/s`，对应左右轮目标速度约为 `20 cm/s`。
