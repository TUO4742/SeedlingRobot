#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "seedling_chassis/mecanum.hpp"
#include "seedling_chassis/serial_port.hpp"

using namespace std::chrono_literals;

namespace seedling_chassis
{

namespace
{
constexpr uint8_t kFrameHead = 0x7B;
constexpr uint8_t kFrameTail = 0x7D;
constexpr size_t kDownLen = 14;   // ROS -> MCU
constexpr size_t kUpLen = 22;     // MCU -> ROS
}  // namespace

class ChassisNode : public rclcpp::Node
{
public:
  ChassisNode() : rclcpp::Node("seedling_chassis")
  {
    port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    rate_hz_ = declare_parameter<double>("rate_hz", 50.0);
    cmd_timeout_ms_ = declare_parameter<int>("cmd_timeout_ms", 300);

    geom_.wheel_base_sum_m = declare_parameter<double>("wheel_base_sum_m", 0.168);
    geom_.wheel_diameter_m = declare_parameter<double>("wheel_diameter_m", 0.065);
    geom_.encoder_line = declare_parameter<double>("encoder_line", 11.0);
    geom_.encoder_multiple = declare_parameter<double>("encoder_multiple", 4.0);
    geom_.gear_ratio = declare_parameter<double>("gear_ratio", 30.0);

    max_vx_ = declare_parameter<double>("max_vx", 0.35);
    max_vy_ = declare_parameter<double>("max_vy", 0.30);
    max_wz_ = declare_parameter<double>("max_wz", 0.80);

    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom_combined");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 20);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_ = *msg;
        last_cmd_time_ = now();
        have_cmd_ = true;
      });

    const double period = 1.0 / std::max(1.0, rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period), std::bind(&ChassisNode::onTimer, this));

    last_tick_ = now();
    RCLCPP_INFO(
      get_logger(), "seedling_chassis started: port=%s baud=%d rate=%.0fHz K=%.3f",
      port_.c_str(), baudrate_, rate_hz_, geom_.wheel_base_sum_m);
  }

private:
  bool ensureSerial()
  {
    if (serial_.isOpen()) {
      return true;
    }
    if (serial_.open(port_, baudrate_)) {
      RCLCPP_INFO(get_logger(), "serial connected: %s", port_.c_str());
      return true;
    }
    return false;
  }

  static uint8_t checksum(const uint8_t * data, size_t len)
  {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
      sum ^= data[i];
    }
    return sum;
  }

  void onTimer()
  {
    const rclcpp::Time tick = now();
    const double dt = std::max(1e-4, (tick - last_tick_).seconds());
    last_tick_ = tick;

    if (!ensureSerial()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "serial %s not available", port_.c_str());
      return;
    }

    std::vector<uint8_t> rx;
    if (serial_.readAvailable(rx, 512) > 0) {
      rx_buf_.insert(rx_buf_.end(), rx.begin(), rx.end());
      parseFrames();
    }

    publishOdom(dt);
    sendCommand();
  }

  /// 滑动窗口定长匹配，与固件 ros_comm_process_rx_bytes() 同思路
  void parseFrames()
  {
    size_t offset = 0;
    while (offset + kUpLen <= rx_buf_.size()) {
      if (rx_buf_[offset] != kFrameHead || rx_buf_[offset + kUpLen - 1] != kFrameTail) {
        ++offset;
        continue;
      }
      if (checksum(rx_buf_.data() + offset, kUpLen - 2) != rx_buf_[offset + kUpLen - 2]) {
        ++offset;
        continue;
      }
      handleFrame(rx_buf_.data() + offset);
      offset += kUpLen;
    }
    if (offset > 0) {
      rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + static_cast<long>(offset));
    }
    if (rx_buf_.size() > 4096) {
      rx_buf_.clear();
    }
  }

  void handleFrame(const uint8_t * f)
  {
    for (int i = 0; i < kWheelCount; ++i) {
      const size_t base = 2 + static_cast<size_t>(i) * 4;
      const int32_t v = static_cast<int32_t>(
        (static_cast<uint32_t>(f[base]) << 24) | (static_cast<uint32_t>(f[base + 1]) << 16) |
        (static_cast<uint32_t>(f[base + 2]) << 8) | static_cast<uint32_t>(f[base + 3]));
      if (!have_first_frame_) {
        enc_last_[i] = v;
      }
      const int32_t delta = static_cast<int32_t>(
        static_cast<uint32_t>(v) - static_cast<uint32_t>(enc_last_[i]));
      enc_delta_[i] += delta;
      enc_last_[i] = v;
    }
    have_first_frame_ = true;
  }

  void publishOdom(double dt)
  {
    std::array<double, kWheelCount> wheel{};
    if (have_first_frame_) {
      wheel = countsToWheelSpeeds(enc_delta_, dt, geom_);
      enc_delta_.fill(0);
    } else {
      wheel.fill(0.0);
    }

    const BodyVelocity body = wheelSpeedsToBody(wheel, geom_);

    const double yaw_mid = yaw_ + body.wz * dt * 0.5;
    x_ += (body.vx * std::cos(yaw_mid) - body.vy * std::sin(yaw_mid)) * dt;
    y_ += (body.vx * std::sin(yaw_mid) + body.vy * std::cos(yaw_mid)) * dt;
    yaw_ += body.wz * dt;
    yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = body.vx;
    odom.twist.twist.linear.y = body.vy;
    odom.twist.twist.angular.z = body.wz;

    // 协方差：编码器里程计，yaw 漂移大，给一个务实的量级
    odom.pose.covariance[0] = 1e-3;
    odom.pose.covariance[7] = 1e-3;
    odom.pose.covariance[35] = 5e-2;
    odom.twist.covariance[0] = 1e-3;
    odom.twist.covariance[7] = 1e-3;
    odom.twist.covariance[35] = 5e-2;

    odom_pub_->publish(odom);
  }

  void sendCommand()
  {
    geometry_msgs::msg::Twist cmd;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      if (have_cmd_ && (now() - last_cmd_time_).milliseconds() > cmd_timeout_ms_) {
        have_cmd_ = false;  // 指令超时，回零
      }
      if (have_cmd_) {
        cmd = cmd_;
      }
    }

    double vx = std::clamp(cmd.linear.x, -max_vx_, max_vx_);
    double vy = std::clamp(cmd.linear.y, -max_vy_, max_vy_);
    double wz = std::clamp(cmd.angular.z, -max_wz_, max_wz_);

    // 麦轮横移效率低，整体按轮速上限等比缩放（与固件归一化策略一致）
    const auto wheels = bodyToWheelSpeeds({vx, vy, wz}, geom_);
    double peak = 0.0;
    for (double w : wheels) {
      peak = std::max(peak, std::fabs(w));
    }
    constexpr double kMaxWheelMps = 0.72;  // robot_config.h: CHASSIS_MAX_WHEEL_SPEED_MPS
    if (peak > kMaxWheelMps) {
      const double scale = kMaxWheelMps / peak;
      vx *= scale;
      vy *= scale;
      wz *= scale;
    }

    uint8_t f[kDownLen];
    std::memset(f, 0, sizeof(f));
    f[0] = kFrameHead;
    f[1] = 0x01;  // bit0 使能
    putFloatBE(f + 2, static_cast<float>(vx));
    putFloatBE(f + 6, static_cast<float>(vy));
    const int16_t wz_milli = static_cast<int16_t>(std::lround(wz * 1000.0));
    f[10] = static_cast<uint8_t>((static_cast<uint16_t>(wz_milli) >> 8) & 0xFF);
    f[11] = static_cast<uint8_t>(static_cast<uint16_t>(wz_milli) & 0xFF);
    f[12] = checksum(f, 12);
    f[13] = kFrameTail;

    std::vector<uint8_t> out(f, f + kDownLen);
    if (!serial_.writeAll(out)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "serial write failed");
    }
  }

  static void putFloatBE(uint8_t * dst, float value)
  {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    dst[0] = static_cast<uint8_t>((bits >> 24) & 0xFF);
    dst[1] = static_cast<uint8_t>((bits >> 16) & 0xFF);
    dst[2] = static_cast<uint8_t>((bits >> 8) & 0xFF);
    dst[3] = static_cast<uint8_t>(bits & 0xFF);
  }

  // --- params ---
  std::string port_;
  int baudrate_ = 115200;
  double rate_hz_ = 50.0;
  int cmd_timeout_ms_ = 300;
  double max_vx_ = 0.35;
  double max_vy_ = 0.30;
  double max_wz_ = 0.80;
  std::string odom_frame_ = "odom_combined";
  std::string base_frame_ = "base_footprint";

  ChassisGeometry geom_;

  // --- state ---
  SerialPort serial_;
  std::vector<uint8_t> rx_buf_;
  std::array<int32_t, kWheelCount> enc_last_{};
  std::array<int32_t, kWheelCount> enc_delta_{};
  bool have_first_frame_ = false;

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;

  geometry_msgs::msg::Twist cmd_;
  rclcpp::Time last_cmd_time_;
  bool have_cmd_ = false;
  std::mutex cmd_mutex_;

  rclcpp::Time last_tick_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace seedling_chassis

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<seedling_chassis::ChassisNode>());
  rclcpp::shutdown();
  return 0;
}
