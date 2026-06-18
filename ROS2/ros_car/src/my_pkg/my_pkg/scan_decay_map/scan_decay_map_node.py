import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


DEFAULT_SCAN_TOPIC = "/scan"
DEFAULT_MAP_TOPIC = "/scan_decay_map"
DEFAULT_MAP_FRAME = "map"


def get_yaw_from_quaternion(rotation):
    """Return yaw from a quaternion."""
    sin_yaw = 2.0 * (
        (rotation.w * rotation.z) + (rotation.x * rotation.y)
    )
    cos_yaw = 1.0 - 2.0 * (
        (rotation.y * rotation.y) + (rotation.z * rotation.z)
    )
    return math.atan2(sin_yaw, cos_yaw)


class ScanDecayMapNode(Node):
    """Publish a time-decayed occupancy overlay from laser hits."""

    def __init__(self):
        super().__init__("scan_decay_map_node")

        self.declare_parameter("scan_topic", DEFAULT_SCAN_TOPIC)
        self.declare_parameter("map_topic", DEFAULT_MAP_TOPIC)
        self.declare_parameter("map_frame", DEFAULT_MAP_FRAME)
        self.declare_parameter("resolution_m", 0.05)
        self.declare_parameter("width_m", 20.0)
        self.declare_parameter("height_m", 20.0)
        self.declare_parameter("origin_x", -10.0)
        self.declare_parameter("origin_y", -10.0)
        self.declare_parameter("decay_seconds", 8.0)
        self.declare_parameter("hit_increment", 25.0)
        self.declare_parameter("max_occupancy", 100.0)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("min_range_m", 0.08)
        self.declare_parameter("max_range_m", 6.0)
        self.declare_parameter("transform_timeout_sec", 0.05)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.map_topic = self.get_parameter("map_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.resolution_m = self.get_parameter("resolution_m").value
        self.origin_x = self.get_parameter("origin_x").value
        self.origin_y = self.get_parameter("origin_y").value
        self.decay_seconds = self.get_parameter("decay_seconds").value
        self.hit_increment = self.get_parameter("hit_increment").value
        self.max_occupancy = self.get_parameter("max_occupancy").value
        self.min_range_m = self.get_parameter("min_range_m").value
        self.max_range_m = self.get_parameter("max_range_m").value
        self.transform_timeout_sec = self.get_parameter(
            "transform_timeout_sec",
        ).value

        width_m = self.get_parameter("width_m").value
        height_m = self.get_parameter("height_m").value
        self.width_cells = int(math.ceil(width_m / self.resolution_m))
        self.height_cells = int(math.ceil(height_m / self.resolution_m))
        cell_count = self.width_cells * self.height_cells
        self.cell_scores = [0.0] * cell_count
        self.cell_update_times = [0.0] * cell_count

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            self.map_topic,
            10,
        )
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )

        publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.create_timer(1.0 / publish_rate_hz, self.publish_decay_map)
        self.get_logger().info(
            f"scan_decay_map_node publishing {self.map_topic}",
        )

    def get_cell_index(self, map_x, map_y):
        """Return the flattened cell index for a map coordinate."""
        cell_x = int((map_x - self.origin_x) / self.resolution_m)
        cell_y = int((map_y - self.origin_y) / self.resolution_m)
        if cell_x < 0 or cell_y < 0:
            return None
        if cell_x >= self.width_cells or cell_y >= self.height_cells:
            return None
        return (cell_y * self.width_cells) + cell_x

    def get_decayed_score(self, cell_index, now_seconds):
        """Return the current score after linear time decay."""
        score = self.cell_scores[cell_index]
        if score <= 0.0:
            return 0.0
        elapsed_seconds = now_seconds - self.cell_update_times[cell_index]
        if elapsed_seconds >= self.decay_seconds:
            return 0.0
        decay_ratio = 1.0 - (elapsed_seconds / self.decay_seconds)
        return score * decay_ratio

    def update_cell_score(self, cell_index, now_seconds):
        """Increase a cell score while preserving elapsed decay."""
        decayed_score = self.get_decayed_score(cell_index, now_seconds)
        updated_score = min(
            self.max_occupancy,
            decayed_score + self.hit_increment,
        )
        self.cell_scores[cell_index] = updated_score
        self.cell_update_times[cell_index] = now_seconds

    def scan_callback(self, scan_message):
        """Project laser scan hits into the map frame."""
        if not scan_message.header.frame_id:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                scan_message.header.frame_id,
                Time.from_msg(scan_message.header.stamp),
                timeout=Duration(seconds=self.transform_timeout_sec),
            )
        except TransformException as error:
            self.get_logger().debug(
                f"scan decay map waiting for TF: {error}",
            )
            return

        now_seconds = self.get_clock().now().nanoseconds / 1_000_000_000.0
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        transform_yaw = get_yaw_from_quaternion(rotation)
        cos_yaw = math.cos(transform_yaw)
        sin_yaw = math.sin(transform_yaw)

        scan_angle = scan_message.angle_min
        for range_m in scan_message.ranges:
            if self.min_range_m <= range_m <= self.max_range_m:
                laser_x = range_m * math.cos(scan_angle)
                laser_y = range_m * math.sin(scan_angle)
                map_x = (
                    translation.x
                    + (laser_x * cos_yaw)
                    - (laser_y * sin_yaw)
                )
                map_y = (
                    translation.y
                    + (laser_x * sin_yaw)
                    + (laser_y * cos_yaw)
                )
                cell_index = self.get_cell_index(map_x, map_y)
                if cell_index is not None:
                    self.update_cell_score(cell_index, now_seconds)
            scan_angle += scan_message.angle_increment

    def build_map_message(self, now_seconds):
        """Build an occupancy grid from decayed cell scores."""
        map_message = OccupancyGrid()
        map_message.header.stamp = self.get_clock().now().to_msg()
        map_message.header.frame_id = self.map_frame
        map_message.info.resolution = float(self.resolution_m)
        map_message.info.width = self.width_cells
        map_message.info.height = self.height_cells
        map_message.info.origin.position.x = float(self.origin_x)
        map_message.info.origin.position.y = float(self.origin_y)
        map_message.info.origin.orientation.w = 1.0

        map_data = []
        for cell_index in range(self.width_cells * self.height_cells):
            decayed_score = self.get_decayed_score(cell_index, now_seconds)
            if decayed_score <= 0.5:
                self.cell_scores[cell_index] = 0.0
                map_data.append(0)
            else:
                map_data.append(int(round(decayed_score)))
        map_message.data = map_data
        return map_message

    def publish_decay_map(self):
        """Publish the current decayed scan-hit overlay."""
        now_seconds = self.get_clock().now().nanoseconds / 1_000_000_000.0
        self.map_publisher.publish(self.build_map_message(now_seconds))


def main(args=None):
    """Run scan_decay_map_node."""
    rclpy.init(args=args)
    node = ScanDecayMapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
