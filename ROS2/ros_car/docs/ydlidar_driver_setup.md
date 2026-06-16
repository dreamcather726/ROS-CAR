# YDLidar Driver Setup

This project follows the Yahboom YDLIDAR X3 Pro ROS2 flow:

1. Install the YDLidar SDK with CMake.
2. Build the `ydlidar_ros2_driver` ROS2 package in the same workspace.
3. Bind the lidar serial device to `/dev/ydlidar`.
4. Start lidar through `ydlidar_ros2_driver`, optionally from `my_pkg` bringup.

## Install SDK

Run this once on Muse Pi from a YDLidar SDK source directory:

```bash
mkdir -p build
cd build
cmake .. -DBUILD_TEST=OFF
make -j4
sudo make install
```

No error during this process means the SDK driver is installed.

After installation, these files should exist:

```bash
ls /usr/local/include/src/CYdLidar.h
ls /usr/local/include/core/common/ydlidar_def.h
ls /usr/local/lib/libydlidar_sdk.a
```

## Build ROS2 Driver Package

This project already includes a ROS2 driver package:

```text
~/ros_car/src/ydlidar_ros2_driver/
```

The workspace should look like this after syncing the project:

```text
~/ros_car/src/
  my_pkg/
  openslam_gmapping/
  slam_gmapping/
  ydlidar_ros2_driver/
```

Build the workspace:

```bash
cd ~/ros_car
colcon build
source install/setup.bash
```

## Bind Lidar Port

The Yahboom driver expects `/dev/ydlidar`. Run the startup script from the
driver package:

```bash
cd ~/ros_car
sudo chmod 777 src/ydlidar_ros2_driver/startup/*
sudo sh src/ydlidar_ros2_driver/startup/initenv.sh
```

Unplug and replug the lidar, then check:

```bash
ll /dev/ydlidar
```

It should point to the real USB serial device, for example `ttyUSB0`.

## Start Lidar

Yahboom standalone test:

```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py
```

Project bringup without RViz:

```bash
ros2 launch my_pkg bringup.launch.py use_lidar:=true
```

Check scan data:

```bash
ros2 topic echo /scan
ros2 topic hz /scan
```

For this car project, keep the lidar frame aligned with the existing TF:

```text
laser_link
```
