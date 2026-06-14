import math

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    import ydlidar
except ImportError:
    ydlidar = None


class YdlidarNode(Node):
    """Publish YDLidar scan data as sensor_msgs/LaserScan."""

    def __init__(self):
        super().__init__("ydlidar_node")
        self.declare_parameter("port", "/dev/ydlidar")
        self.declare_parameter("frame_id", "laser_link")
        self.declare_parameter("baudrate", 230400)
        self.declare_parameter("scan_frequency_hz", 10.0)
        self.declare_parameter("sample_rate", 5)
        self.declare_parameter("min_angle_deg", -180.0)
        self.declare_parameter("max_angle_deg", 180.0)
        self.declare_parameter("min_range_m", 0.08)
        self.declare_parameter("max_range_m", 16.0)
        self.declare_parameter("single_channel", False)
        self.declare_parameter("inverted", False)
        self.declare_parameter("reversion", False)
        self.declare_parameter("intensity", False)

        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.publisher = self.create_publisher(LaserScan, "/scan", 10)
        self.laser = None
        self.sdk_scan = None
        self.timer = None

        if ydlidar is None:
            self.get_logger().error(
                "Cannot import ydlidar. Build and install YDLidar-SDK first."
            )
            return

        self._start_lidar()

    def _start_lidar(self):
        """Configure SDK options and start scanning."""
        ydlidar.os_init()
        self.laser = ydlidar.CYdLidar()
        self.sdk_scan = ydlidar.LaserScan()

        port = self.get_parameter("port").get_parameter_value().string_value
        baudrate = (
            self.get_parameter("baudrate").get_parameter_value().integer_value
        )
        scan_frequency_hz = (
            self.get_parameter("scan_frequency_hz")
            .get_parameter_value()
            .double_value
        )
        sample_rate = (
            self.get_parameter("sample_rate")
            .get_parameter_value()
            .integer_value
        )
        min_angle_deg = (
            self.get_parameter("min_angle_deg")
            .get_parameter_value()
            .double_value
        )
        max_angle_deg = (
            self.get_parameter("max_angle_deg")
            .get_parameter_value()
            .double_value
        )
        min_range_m = (
            self.get_parameter("min_range_m")
            .get_parameter_value()
            .double_value
        )
        max_range_m = (
            self.get_parameter("max_range_m")
            .get_parameter_value()
            .double_value
        )
        single_channel = (
            self.get_parameter("single_channel")
            .get_parameter_value()
            .bool_value
        )
        inverted = (
            self.get_parameter("inverted").get_parameter_value().bool_value
        )
        reversion = (
            self.get_parameter("reversion").get_parameter_value().bool_value
        )
        intensity = (
            self.get_parameter("intensity").get_parameter_value().bool_value
        )

        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, baudrate)
        self.laser.setlidaropt(
            ydlidar.LidarPropLidarType,
            ydlidar.TYPE_TRIANGLE,
        )
        self.laser.setlidaropt(
            ydlidar.LidarPropDeviceType,
            ydlidar.YDLIDAR_TYPE_SERIAL,
        )
        self.laser.setlidaropt(
            ydlidar.LidarPropScanFrequency,
            scan_frequency_hz,
        )
        self.laser.setlidaropt(ydlidar.LidarPropSampleRate, sample_rate)
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, single_channel)
        self.laser.setlidaropt(ydlidar.LidarPropInverted, inverted)
        self.laser.setlidaropt(ydlidar.LidarPropReversion, reversion)
        self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, max_angle_deg)
        self.laser.setlidaropt(ydlidar.LidarPropMinAngle, min_angle_deg)
        self.laser.setlidaropt(ydlidar.LidarPropMaxRange, max_range_m)
        self.laser.setlidaropt(ydlidar.LidarPropMinRange, min_range_m)
        self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, intensity)
        self.laser.setlidaropt(ydlidar.LidarPropAutoReconnect, True)

        if not self.laser.initialize():
            self.get_logger().error(self.laser.DescribeError())
            return
        if not self.laser.turnOn():
            self.get_logger().error(self.laser.DescribeError())
            return

        self.timer = self.create_timer(0.01, self._publish_scan)
        self.get_logger().info(
            f"ydlidar_node started on {port}, publishing /scan"
        )

    def _publish_scan(self):
        """Read one SDK scan and publish it as a ROS LaserScan message."""
        if self.laser is None or self.sdk_scan is None:
            return

        if not self.laser.doProcessSimple(self.sdk_scan):
            self.get_logger().warn(
                "Failed to get lidar data",
                throttle_duration_sec=2.0,
            )
            return

        scan_msg = self._build_scan_message(self.sdk_scan)
        self.publisher.publish(scan_msg)

    def _build_scan_message(self, sdk_scan):
        """Convert a YDLidar SDK scan into a ROS LaserScan message."""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self._build_ros_time(sdk_scan.stamp)
        scan_msg.header.frame_id = self.frame_id
        scan_msg.angle_min = sdk_scan.config.min_angle
        scan_msg.angle_max = sdk_scan.config.max_angle
        scan_msg.angle_increment = sdk_scan.config.angle_increment
        scan_msg.time_increment = sdk_scan.config.time_increment
        scan_msg.scan_time = sdk_scan.config.scan_time
        scan_msg.range_min = sdk_scan.config.min_range
        scan_msg.range_max = sdk_scan.config.max_range

        point_count = self._calculate_point_count(sdk_scan.config)
        scan_msg.ranges = [math.inf] * point_count
        scan_msg.intensities = [0.0] * point_count

        for point in sdk_scan.points:
            index = math.ceil(
                (point.angle - sdk_scan.config.min_angle)
                / sdk_scan.config.angle_increment
            )
            if 0 <= index < point_count:
                scan_msg.ranges[index] = point.range
                scan_msg.intensities[index] = point.intensity

        return scan_msg

    def _calculate_point_count(self, scan_config):
        """Calculate output array length from SDK scan configuration."""
        angle_range = scan_config.max_angle - scan_config.min_angle
        if scan_config.angle_increment <= 0.0 or angle_range <= 0.0:
            return 0
        return int(angle_range / scan_config.angle_increment) + 1

    def _build_ros_time(self, stamp_nanoseconds):
        """Convert SDK nanosecond timestamp to a ROS Time message."""
        if stamp_nanoseconds <= 0:
            return self.get_clock().now().to_msg()

        stamp = Time()
        stamp.sec = int(stamp_nanoseconds // 1000000000)
        stamp.nanosec = int(stamp_nanoseconds % 1000000000)
        return stamp

    def destroy_node(self):
        """Stop the lidar before shutting down the ROS node."""
        if self.laser is not None:
            self.laser.turnOff()
            self.laser.disconnecting()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YdlidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
