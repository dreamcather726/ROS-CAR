# Third Party Code

This directory stores third-party source packages used by `my_pkg`.

## ydlidar_sdk

`ydlidar_sdk` contains the YDLidar SDK source code for YDLIDAR X3 Pro.
The ROS2 node in `my_pkg.lidar.ydlidar_node` imports the SDK Python module:

```bash
python3 -c "import ydlidar"
```

If the import fails on Muse Pi, install the SDK Python binding from this
directory before starting `ydlidar_node`:

```bash
cd ~/ros_car/src/my_pkg/third_party/ydlidar_sdk
pip install .
```
