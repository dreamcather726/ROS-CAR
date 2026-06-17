# ROS2 小车建图操作步骤

本文记录 Muse Pi 上 ROS2 小车从导入工程到完成 GMapping 建图的完整操作流程。

## 1. 导入全部文件

把 ROS2 小车工程完整放到 Muse Pi，例如：

```bash
~/ros2_car
```

ROS2 工作区目录建议保持下面的结构：

```text
~/ros2_car/
  src/
    my_pkg/
    openslam_gmapping/
    slam_gmapping/
    ydlidar_ros2_driver/
```

`ydlidar_ros2_driver` 是 ROS2 驱动包，需要放在 `src/` 下。

`YDLidar-SDK` 不是 ROS2 包，不建议放进 `src/`。建议单独放在：

```bash
~/YDLidar-SDK
```

## 2. 安装 YDLidar SDK

进入 SDK 源码目录并编译安装：

```bash
cd ~/YDLidar-SDK
mkdir -p build
cd build
cmake .. -DBUILD_TEST=OFF
make -j4
sudo make install
```

`-DBUILD_TEST=OFF` 用来关闭 SDK 自带测试，避免 gtest 或 C++ 标准版本导致无关编译失败。

安装完成后检查 SDK 头文件和库是否存在：

```bash
ls /usr/local/include/src/CYdLidar.h
ls /usr/local/lib/libydlidar_sdk.a
```

如果这两个文件都能看到，说明 YDLidar SDK 安装成功。

如果提示 `No such file or directory`，说明 SDK 没有安装成功，需要回到 `~/YDLidar-SDK/build` 重新执行：

```bash
cmake .. -DBUILD_TEST=OFF
make -j4
sudo make install
```

## 3. 安装 pyserial

底盘 ESP32 串口通信需要 Python 串口库 `pyserial`。

优先使用系统 apt 安装：

```bash
sudo apt update
sudo apt install -y python3-serial
```

安装后检查 Python 是否能导入 `serial`：

```bash
python3 -c "import serial; print(serial.__version__)"
```

如果 `apt` 源里没有 `python3-serial`，再使用 `pip` 安装：

```bash
python3 -m pip install --user pyserial
```

## 4. 配置串口权限

先查看当前串口设备：

```bash
ls -l /dev/ttyUSB*
```

当前实车串口分配：

```text
底盘 ESP32: /dev/ttyUSB1
激光雷达:   /dev/ttyUSB0
```

底盘和雷达不能使用同一个串口设备。

如果启动底盘或雷达时提示没有串口权限，例如：

```text
Permission denied: '/dev/ttyUSB0'
Permission denied: '/dev/ttyUSB1'
```

可以先用临时方式给当前设备加权限：

```bash
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
```

这个方法重启或重新插拔 USB 后可能失效，适合临时测试。

长期方式是把当前用户加入 `dialout` 组：

```bash
sudo usermod -aG dialout $USER
```

执行后需要退出 SSH 或重启 Muse Pi，让组权限重新加载：

```bash
sudo reboot
```

重新登录后检查当前用户是否已经在 `dialout` 组：

```bash
groups
```

如果输出里能看到 `dialout`，通常就可以直接访问 `/dev/ttyUSB0` 和 `/dev/ttyUSB1`。

## 5. 编译 ROS2 工作区环境

回到 ROS2 工作区根目录：

```bash
cd ~/ros2_car
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

`--symlink-install` 的作用是让 Python、launch、yaml 等文件在 `install/` 中尽量使用软链接。开发调试时，修改这些文件后通常只需要重新启动节点，不一定每次都重新编译。

编译完成后加载工作区环境：

```bash
source install/setup.bash
```

如果最后看到类似输出：

```text
Summary: 4 packages finished
```

说明工作区包已经编译完成。

如果修改了 C++ 源码，例如 `ydlidar_ros2_driver` 或 `slam_gmapping` 的 `.cpp` 文件，需要重新执行：

```bash
colcon build --symlink-install
source install/setup.bash
```

## 6. 启动底盘和雷达

终端 1 启动底盘、TF 和雷达，不启动 GMapping：

```bash
cd ~/ros2_car
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch my_pkg bringup.launch.py use_lidar:=true use_gmapping:=false port:=/dev/ttyUSB1 lidar_port:=/dev/ttyUSB0
```

这个命令会启动：

```text
esp32_bridge_node
tf_tree_node
ydlidar_ros2_driver_node
```

## 7. 检查底盘、雷达和 TF 数据

另开终端检查 `/scan`、`/odom` 和 TF：

```bash
cd ~/ros2_car
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_link
```

GMapping 需要下面这条 TF 链正常：

```text
odom -> base_link -> laser_link
```

如果出现：

```text
Message Filter dropping message: frame 'laser_link'
discarding message because the queue is full
```

通常说明 `/scan` 有数据，但 GMapping 等不到匹配时间戳的 TF。优先检查 `/odom` 是否有频率，以及两个 `tf2_echo` 是否正常。

## 8. 启动 GMapping 建图

确认 `/scan`、`/odom` 和 TF 稳定后，另开终端启动 GMapping：

```bash
cd ~/ros2_car
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run slam_gmapping slam_gmapping
```

也可以使用项目里的 launch 文件启动：

```bash
ros2 launch slam_gmapping gmapping_x3_launch.py
```

如果能看到类似输出，说明 GMapping 已经开始处理雷达数据：

```text
Registering First Scan
Registering Scans:Done
```

## 9. 启动键盘控制

另开终端启动键盘控制节点：

```bash
cd ~/ros2_car
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run my_pkg keyboard_control_node
```

按键说明：

```text
W: 前进
S: 后退
A: 左转
D: 右转
Space 或 X: 停止
```

键盘控制节点所在终端需要保持在前台，按键才会被节点接收。建图时建议慢速移动小车，避免快速原地旋转，否则地图容易拉伸或变形。

## 10. 用 RViz 查看地图

另开终端启动 RViz：

```bash
rviz2
```

RViz 中设置：

```text
Fixed Frame: map
Add -> TF
Add -> By topic -> /scan -> LaserScan
Add -> By topic -> /map -> Map
```

如果地图刷新太慢，可以临时提高 GMapping 的 `/map` 发布频率：

```bash
ros2 run slam_gmapping slam_gmapping --ros-args -p map_update_interval:=0.5
```

查看 `/map` 实际刷新频率：

```bash
ros2 topic hz /map
```

## 11. 保存地图

地图看起来可用后保存：

```bash
mkdir -p ~/ros2_car/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros2_car/maps/first_map
```

生成文件：

```text
~/ros2_car/maps/first_map.yaml
~/ros2_car/maps/first_map.pgm
```

