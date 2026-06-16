# GMapping Workflow

This file records the working 2D mapping flow for this robot.

## Package Roles

Use the packages like this:

```text
ydlidar_ros2_driver  -> publishes /scan
my_pkg               -> publishes /odom and robot TF
openslam_gmapping    -> gmapping algorithm library
slam_gmapping        -> consumes /scan + TF and publishes /map
```

Do not start tutorial odom, tutorial transform, or extra
`static_transform_publisher` nodes from downloaded examples. This project uses
its own chassis odometry.

The required TF chain is:

```text
odom -> base_link -> laser_link
```

## Build

Build the whole workspace:

```bash
cd ~/ros_car
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If you want to build packages one by one:

```bash
cd ~/ros_car
source /opt/ros/humble/setup.bash

colcon build --packages-select openslam_gmapping --symlink-install
source install/setup.bash

colcon build --packages-select ydlidar_ros2_driver --symlink-install
source install/setup.bash

colcon build --packages-select slam_gmapping --symlink-install
source install/setup.bash

colcon build --packages-select my_pkg --symlink-install
source install/setup.bash
```

If `source install/setup.bash` prints a warning about a missing package path
while you are building one package at a time, finish rebuilding all packages or
open a new terminal and source only `/opt/ros/humble/setup.bash` before building.

## Start Mapping

Start chassis, TF, lidar, and gmapping together:

```bash
cd ~/ros_car
source install/setup.bash
ros2 launch my_pkg bringup.launch.py use_lidar:=true use_gmapping:=true port:=/dev/ttyUSB1 lidar_port:=/dev/ydlidar
```

Current serial assignment:

```text
Chassis ESP32: /dev/ttyUSB1
YDLidar:       /dev/ydlidar
```

If the devices changed, check them first:

```bash
ls -l /dev/ttyUSB*
ls -l /dev/ydlidar
```

Make sure the chassis and lidar do not use the same real serial device.

## Check Nodes

In another terminal:

```bash
cd ~/ros_car
source install/setup.bash
ros2 node list
```

Expected nodes:

```text
/esp32_bridge_node
/tf_tree_node
/ydlidar_ros2_driver_node
/slam_gmapping
```

## Check Topics

Check lidar, odometry, and map data:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /map --once
```

Expected result:

```text
/scan has data from ydlidar_ros2_driver_node.
/odom has data from esp32_bridge_node.
/map has width, height, and resolution from slam_gmapping.
```

For `/odom`, confirm these frame names:

```bash
ros2 topic echo /odom --once
```

Expected frame fields:

```yaml
header:
  frame_id: odom
child_frame_id: base_link
```

## Check TF

GMapping needs time-matched TF. Check both transforms:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_link
```

If `tf2_echo odom base_link` says `frame does not exist`, then `/odom` is not
being published or `tf_tree_node` is not receiving it. Check the ESP32 serial
port and `/esp32/status`.

```bash
ros2 topic echo /esp32/status --once
ros2 topic hz /odom
```

## RViz

Start RViz:

```bash
rviz2
```

Set:

```text
Fixed Frame: map
```

Add displays:

```text
Add -> TF
Add -> By topic -> /scan -> LaserScan
Add -> By topic -> /map -> Map
```

If `Map` shows `No map received`, keep RViz open and move the robot slowly, or
restart the mapping launch while RViz is already open.

## Move Slowly While Mapping

Move slowly. Fast turns, wheel slip, or unstable odometry will stretch the map.

Slow forward:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.03}, angular: {z: 0.0}}" -r 5
```

Slow turn:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.2}}" -r 5
```

Stop:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

## Save Map

After the map looks usable:

```bash
mkdir -p ~/ros_car/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros_car/maps/first_map
```

Generated files:

```text
~/ros_car/maps/first_map.yaml
~/ros_car/maps/first_map.pgm
```

## Troubleshooting

If `slam_gmapping` prints:

```text
Message Filter dropping message: frame 'laser_link'
discarding message because the queue is full
```

Check:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_link
```

The usual reason is that gmapping receives `/scan`, but cannot find the matching
`odom -> base_link -> laser_link` TF at the scan timestamp.

If `/map` exists in terminal but RViz does not show it:

```bash
ros2 topic echo /map --once
```

Then remove and re-add the RViz Map display:

```text
Add -> By topic -> /map -> Map
```

If the map looks stretched or fan-shaped:

```text
Move slower.
Avoid fast in-place turns.
Check /odom frequency and smoothness.
Check /scan frequency.
Check lidar inverted/reversion parameters.
```
