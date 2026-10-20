#ifndef SEEDLING_CHASSIS__MECANUM_HPP_
#define SEEDLING_CHASSIS__MECANUM_HPP_

#include <array>

namespace seedling_chassis
{

/// 轮序与固件一致：0=FL, 1=RR, 2=RL, 3=FR
enum WheelIndex : int
{
  kWheelFL = 0,
  kWheelRR = 1,
  kWheelRL = 2,
  kWheelFR = 3,
  kWheelCount = 4,
};

/// 与固件 robot_config.h 保持一致
struct ChassisGeometry
{
  /// 轴距 + 轮距，单位 m（robot_config.h: CHASSIS_WHEEL_BASE_SUM_M）
  double wheel_base_sum_m = 0.168;
  /// 轮直径，单位 m（motor.h: WHEEL_DIAMETER）
  double wheel_diameter_m = 0.065;
  /// 编码器线数（motor.h: ENC_LINE）
  double encoder_line = 11.0;
  /// 四倍频（motor.h: ENC_MULTIPLE）
  double encoder_multiple = 4.0;
  /// 减速比（motor.h: GEAR_RATIO）
  double gear_ratio = 30.0;

  /// 每轮转一圈的计数
  double counts_per_rev() const { return encoder_line * encoder_multiple * gear_ratio; }
  /// 轮半径
  double wheel_radius() const { return wheel_diameter_m * 0.5; }
  /// 轮周长
  double wheel_circumference() const { return 2.0 * 3.14159265358979323846 * wheel_radius(); }
};

struct BodyVelocity
{
  double vx = 0.0;  // m/s，正=前进
  double vy = 0.0;  // m/s，正=向左
  double wz = 0.0;  // rad/s，正=逆时针
};

/// 编码器增量 → 四轮轮缘线速度（m/s）
/// delta_counts 为本次采样内的计数增量，dt 为采样周期
std::array<double, kWheelCount> countsToWheelSpeeds(
  const std::array<int32_t, kWheelCount> & delta_counts,
  double dt,
  const ChassisGeometry & geom);

/// 四轮线速度 → 车体速度（与固件 encoder.c 的正解一致）
BodyVelocity wheelSpeedsToBody(
  const std::array<double, kWheelCount> & wheel_speeds,
  const ChassisGeometry & geom);

/// 车体速度 → 四轮线速度（与固件 chassis_task.c:169-172 一致）
/// ROS 侧只用于限幅校核，不参与下发
std::array<double, kWheelCount> bodyToWheelSpeeds(
  const BodyVelocity & body,
  const ChassisGeometry & geom);

}  // namespace seedling_chassis

#endif  // SEEDLING_CHASSIS__MECANUM_HPP_
