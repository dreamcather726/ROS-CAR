# Lidar View And Mapping

This file records the current steps for viewing the upside-down YDLidar and
building a 2D map.

## Lidar Mounting

The lidar is mounted upside down on this robot.

The driver parameter file is:

```text
src/ydlidar_ros2_driver/params/ydlidar.yaml
```

Current recommended parameters:

```yaml
baudrate: 115200
sample_rate: 3
single_channel: true
inverted: true
reversion: false
frame_id: "laser_link"
```

Use `inverted: true` for the upside-down mounting. If the scan appears mirrored
left-to-right in RViz, keep `inverted: true` and test `reversion: true`.

## Start Chassis And Lidar

Build after changing parameters:

```bash
cd ~/ros_car
colcon build
source install/setup.bash
```

Start chassis and lidar together:

```bash
ros2 launch my_pkg bringup.launch.py use_lidar:=true port:=/dev/ttyUSB1 lidar_port:=/dev/ydlidar
```

If the chassis is not on `/dev/ttyUSB1`, replace only the `port:=...` value.

## Verify Scan Data

Check the scan topic:

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

The scan frame should be:

```text
laser_link
```

## View Lidar Points In RViz

If using a desktop with display support, open RViz:

```bash
rviz2
```

In RViz:

```text
Fixed Frame: base_link
Add -> By topic -> /scan -> LaserScan
Add -> By display type -> TF
```

If RViz runs on another computer, make sure that computer has the same ROS
domain and can discover the Muse Pi topics.

For SSH/headless Muse Pi, do not use `ydlidar_launch_view.py`; it starts RViz
and will fail without a display.

## Build A 2D Map With slam_gmapping

The mapping packages are included in this workspace:

```text
src/openslam_gmapping
src/slam_gmapping
src/ydlidar_ros2_driver
```

Their roles in this project are:

```text
ydlidar_ros2_driver: reads the YDLidar and publishes /scan.
openslam_gmapping: algorithm library used by slam_gmapping.
slam_gmapping: subscribes to /scan, uses TF odom -> base_link -> laser_link,
and publishes /map plus map -> odom.
```

Do not start the tutorial transform or odom examples from the downloaded
packages. This project uses its own chassis odometry:

```text
my_pkg esp32_bridge_node publishes /odom.
my_pkg tf_tree_node converts /odom into odom -> base_link TF.
my_pkg tf_tree_node publishes base_link -> laser_link static TF.
```

Terminal 1, start the robot, lidar, and gmapping together:

```bash
cd ~/ros_car
source install/setup.bash
ros2 launch my_pkg bringup.launch.py use_lidar:=true use_gmapping:=true port:=/dev/ttyUSB1 lidar_port:=/dev/ydlidar
```

Or start gmapping in a separate terminal after the robot and lidar are already
running:

```bash
source ~/ros_car/install/setup.bash
ros2 launch slam_gmapping gmapping_x3_launch.py
```

Terminal 2 or RViz, verify map data:

```bash
ros2 topic hz /map
ros2 topic echo /map --once
```

In RViz:

```text
Fixed Frame: map
Add -> By topic -> /map -> Map
Add -> By topic -> /scan -> LaserScan
Add -> By display type -> TF
```

Move the robot slowly while mapping. Fast turns or wheel slip will make the map
twist or blur.

## Build A 2D Map With SLAM Toolbox

Install SLAM Toolbox if it is not available:

```bash
sudo apt install ros-humble-slam-toolbox
```

Terminal 1, start the robot:

```bash
cd ~/ros_car
source install/setup.bash
ros2 launch my_pkg bringup.launch.py use_lidar:=true port:=/dev/ttyUSB1 lidar_port:=/dev/ydlidar
```

Terminal 2, start online SLAM:

```bash
source /opt/ros/humble/setup.bash
ros2 launch slam_toolbox online_async_launch.py
```

Terminal 3 or RViz, verify map data:

```bash
ros2 topic hz /map
ros2 topic echo /map --once
```

In RViz:

```text
Fixed Frame: map
Add -> By topic -> /map -> Map
Add -> By topic -> /scan -> LaserScan
Add -> By display type -> TF
```

Move the robot slowly while mapping. Fast turns or wheel slip will make the map
twist or blur.

## Save The Map

After the map looks good:

```bash
mkdir -p ~/ros_car/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros_car/maps/first_map
```

This creates:

```text
~/ros_car/maps/first_map.yaml
~/ros_car/maps/first_map.pgm
```

## Troubleshooting

If `/scan` points appear behind the robot when the obstacle is in front:

```text
Check laser frame orientation and test reversion: true.
```

If `/map` does not appear:

```bash
ros2 topic list | grep scan
ros2 topic list | grep map
ros2 run tf2_ros tf2_echo base_link laser_link
ros2 run tf2_ros tf2_echo odom base_link
```

Mapping needs these transforms to be available:

```text
odom -> base_link
base_link -> laser_link
```
