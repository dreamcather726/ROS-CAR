#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "CYdLidar.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class YdlidarRos2DriverNode : public rclcpp::Node {
 public:
  YdlidarRos2DriverNode()
  : Node("ydlidar_ros2_driver_node") {
    declare_parameter<std::string>("port", "/dev/ydlidar");
    declare_parameter<std::string>("frame_id", "laser_link");
    declare_parameter<int>("baudrate", 230400);
    declare_parameter<int>("sample_rate", 5);
    declare_parameter<int>("abnormal_check_count", 4);
    declare_parameter<double>("scan_frequency_hz", 10.0);
    declare_parameter<double>("min_angle_deg", -180.0);
    declare_parameter<double>("max_angle_deg", 180.0);
    declare_parameter<double>("min_range_m", 0.08);
    declare_parameter<double>("max_range_m", 16.0);
    declare_parameter<bool>("fixed_resolution", false);
    declare_parameter<bool>("reversion", false);
    declare_parameter<bool>("inverted", false);
    declare_parameter<bool>("auto_reconnect", true);
    declare_parameter<bool>("single_channel", false);
    declare_parameter<bool>("intensity", false);
    declare_parameter<bool>("support_motor_dtr", true);
    declare_parameter<bool>("support_heartbeat", false);

    frame_id_ = get_parameter("frame_id").as_string();
    scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    ydlidar::os_init();
    configureLidar();

    if (!laser_.initialize()) {
      RCLCPP_ERROR(get_logger(), "YDLidar initialize failed: %s",
                   laser_.DescribeError());
      return;
    }

    if (!laser_.turnOn()) {
      RCLCPP_ERROR(get_logger(), "YDLidar turnOn failed: %s",
                   laser_.DescribeError());
      return;
    }

    is_lidar_running_ = true;
    scan_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&YdlidarRos2DriverNode::publishScan, this));

    RCLCPP_INFO(get_logger(), "YDLidar ROS2 driver started, publishing /scan");
  }

  ~YdlidarRos2DriverNode() override {
    if (is_lidar_running_) {
      laser_.turnOff();
    }
    laser_.disconnecting();
  }

 private:
  void configureLidar() {
    const std::string port = get_parameter("port").as_string();
    const int baudrate = get_parameter("baudrate").as_int();
    const int sample_rate = get_parameter("sample_rate").as_int();
    const int abnormal_check_count =
      get_parameter("abnormal_check_count").as_int();
    const float scan_frequency_hz =
      static_cast<float>(get_parameter("scan_frequency_hz").as_double());
    const float min_angle_deg =
      static_cast<float>(get_parameter("min_angle_deg").as_double());
    const float max_angle_deg =
      static_cast<float>(get_parameter("max_angle_deg").as_double());
    const float min_range_m =
      static_cast<float>(get_parameter("min_range_m").as_double());
    const float max_range_m =
      static_cast<float>(get_parameter("max_range_m").as_double());
    const bool fixed_resolution =
      get_parameter("fixed_resolution").as_bool();
    const bool reversion = get_parameter("reversion").as_bool();
    const bool inverted = get_parameter("inverted").as_bool();
    const bool auto_reconnect = get_parameter("auto_reconnect").as_bool();
    const bool single_channel = get_parameter("single_channel").as_bool();
    const bool intensity = get_parameter("intensity").as_bool();
    const bool support_motor_dtr =
      get_parameter("support_motor_dtr").as_bool();
    const bool support_heartbeat =
      get_parameter("support_heartbeat").as_bool();

    setStringOption(LidarPropSerialPort, port);
    setIntOption(LidarPropSerialBaudrate, baudrate);
    setIntOption(LidarPropLidarType, TYPE_TRIANGLE);
    setIntOption(LidarPropDeviceType, YDLIDAR_TYPE_SERIAL);
    setIntOption(LidarPropSampleRate, sample_rate);
    setIntOption(LidarPropAbnormalCheckCount, abnormal_check_count);
    setBoolOption(LidarPropFixedResolution, fixed_resolution);
    setBoolOption(LidarPropReversion, reversion);
    setBoolOption(LidarPropInverted, inverted);
    setBoolOption(LidarPropAutoReconnect, auto_reconnect);
    setBoolOption(LidarPropSingleChannel, single_channel);
    setBoolOption(LidarPropIntenstiy, intensity);
    setBoolOption(LidarPropSupportMotorDtrCtrl, support_motor_dtr);
    setBoolOption(LidarPropSupportHeartBeat, support_heartbeat);
    setFloatOption(LidarPropScanFrequency, scan_frequency_hz);
    setFloatOption(LidarPropMinAngle, min_angle_deg);
    setFloatOption(LidarPropMaxAngle, max_angle_deg);
    setFloatOption(LidarPropMinRange, min_range_m);
    setFloatOption(LidarPropMaxRange, max_range_m);
  }

  void setStringOption(int option_name, const std::string & option_value) {
    laser_.setlidaropt(
      option_name,
      option_value.c_str(),
      static_cast<int>(option_value.size()));
  }

  void setIntOption(int option_name, int option_value) {
    laser_.setlidaropt(option_name, &option_value, sizeof(option_value));
  }

  void setBoolOption(int option_name, bool option_value) {
    laser_.setlidaropt(option_name, &option_value, sizeof(option_value));
  }

  void setFloatOption(int option_name, float option_value) {
    laser_.setlidaropt(option_name, &option_value, sizeof(option_value));
  }

  void publishScan() {
    LaserScan sdk_scan;
    if (!laser_.doProcessSimple(sdk_scan)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Failed to get YDLidar data: %s",
        laser_.DescribeError());
      return;
    }

    auto scan_message = sensor_msgs::msg::LaserScan();
    scan_message.header.stamp = buildRosTime(sdk_scan.stamp);
    scan_message.header.frame_id = frame_id_;
    scan_message.angle_min = sdk_scan.config.min_angle;
    scan_message.angle_max = sdk_scan.config.max_angle;
    scan_message.angle_increment = sdk_scan.config.angle_increment;
    scan_message.time_increment = sdk_scan.config.time_increment;
    scan_message.scan_time = sdk_scan.config.scan_time;
    scan_message.range_min = sdk_scan.config.min_range;
    scan_message.range_max = sdk_scan.config.max_range;

    const int point_count = calculatePointCount(sdk_scan.config);
    if (point_count <= 0) {
      return;
    }

    scan_message.ranges.assign(
      point_count,
      std::numeric_limits<float>::infinity());
    scan_message.intensities.assign(point_count, 0.0F);

    for (const auto & point : sdk_scan.points) {
      const int index = static_cast<int>(std::ceil(
        (point.angle - sdk_scan.config.min_angle) /
        sdk_scan.config.angle_increment));
      if (index >= 0 && index < point_count) {
        scan_message.ranges[index] = point.range;
        scan_message.intensities[index] = point.intensity;
      }
    }

    scan_publisher_->publish(scan_message);
  }

  builtin_interfaces::msg::Time buildRosTime(uint64_t stamp_nanoseconds) {
    if (stamp_nanoseconds == 0) {
      stamp_nanoseconds = static_cast<uint64_t>(now().nanoseconds());
    }

    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(stamp_nanoseconds / 1000000000ULL);
    stamp.nanosec = static_cast<uint32_t>(stamp_nanoseconds % 1000000000ULL);
    return stamp;
  }

  int calculatePointCount(const LaserConfig & scan_config) {
    const float angle_range = scan_config.max_angle - scan_config.min_angle;
    if (scan_config.angle_increment <= 0.0F || angle_range <= 0.0F) {
      return 0;
    }
    return static_cast<int>(angle_range / scan_config.angle_increment) + 1;
  }

  CYdLidar laser_;
  std::string frame_id_;
  bool is_lidar_running_{false};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<YdlidarRos2DriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
