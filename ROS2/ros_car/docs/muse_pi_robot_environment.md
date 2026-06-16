# Muse Pi Robot Environment

This file records the current working setup for the ROS2 robot on Muse Pi.

## Workspace

The project workspace is:

```bash
~/ros_car
```

The expected source layout is:

```text
~/ros_car/src/
  my_pkg/
  openslam_gmapping/
  slam_gmapping/
  ydlidar_ros2_driver/
```

## Serial Ports

Do not let the chassis and lidar use the same serial device.

Current recommended assignment:

```text
Chassis ESP32: /dev/ttyUSB1
YDLidar:       /dev/ydlidar
```

`/dev/ydlidar` is a udev alias. It may point to `ttyUSB0` or `ttyUSB1`
internally, but launch commands should use `/dev/ydlidar` for the lidar.

Check the current devices:

```bash
ls -l /dev/ttyUSB*
ls -l /dev/ydlidar
```

If `/dev/ydlidar` points to the same real device as the ESP32 port, swap the
ESP32 `port:=...` value or replug the devices and check again.

## Install YDLidar SDK

Run once from a YDLidar SDK source directory:

```bash
mkdir -p build
cd build
cmake .. -DBUILD_TEST=OFF
make -j4
sudo make install
```

Verify the SDK installation:

```bash
ls /usr/local/include/src/CYdLidar.h
ls /usr/local/include/core/common/ydlidar_def.h
ls /usr/local/lib/libydlidar_sdk.a
```

## Bind YDLidar Alias

Install the udev rule:

```bash
cd ~/ros_car
sudo chmod 777 src/ydlidar_ros2_driver/startup/*
sudo sh src/ydlidar_ros2_driver/startup/initenv.sh
```

Unplug and replug the lidar, then check:

```bash
ls -l /dev/ydlidar
```

## Build ROS2 Workspace

Build after code or launch changes:

```bash
cd ~/ros_car
colcon build
source install/setup.bash
```

For every new terminal:

```bash
cd ~/ros_car
source install/setup.bash
```

## Start Chassis And Lidar

Default one-command startup:

```bash
ros2 launch my_pkg bringup.launch.py use_lidar:=true
```

This starts:

```text
esp32_bridge_node          Chassis ESP32 bridge
tf_tree_node               Robot TF tree
ydlidar_ros2_driver_node   Lidar /scan publisher
```

Explicit one-command startup with both serial ports:

```bash
ros2 launch my_pkg bringup.launch.py use_lidar:=true port:=/dev/ttyUSB1 lidar_port:=/dev/ydlidar
```

Use this form when testing different serial assignments:

```bash
ros2 launch my_pkg bringup.launch.py use_lidar:=true port:=/dev/ttyUSB0 lidar_port:=/dev/ydlidar
```

## Verify Topics

After startup, check core topics:

```bash
ros2 topic list
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic echo /esp32/status --once
```

Expected publishers:

```text
/odom          from esp32_bridge_node
/imu/rawdata   from esp32_bridge_node
/imu/data      from esp32_bridge_node
/esp32/status  from esp32_bridge_node
/tf            from tf_tree_node
/tf_static     from tf_tree_node
/scan          from ydlidar_ros2_driver_node
```

## Common Problems

If `esp32_bridge_node` prints this error:

```text
device reports readiness to read but returned no data
```

First check whether the chassis and lidar are using the same serial device.
Only one process can safely use one serial port.

If the lidar starts but the chassis fails, use:

```bash
ls -l /dev/ydlidar
ls -l /dev/ttyUSB*
```

Then start with explicit serial parameters:

```bash
ros2 launch my_pkg bringup.launch.py use_lidar:=true port:=/dev/ttyUSB0 lidar_port:=/dev/ydlidar
```

If RViz fails with `could not connect to display`, use `bringup.launch.py` or
`ydlidar_launch.py` instead of `ydlidar_launch_view.py` on SSH/headless runs.
