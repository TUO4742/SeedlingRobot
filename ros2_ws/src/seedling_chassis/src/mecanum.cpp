#include "seedling_chassis/mecanum.hpp"

#include <cmath>

namespace seedling_chassis
{

std::array<double, kWheelCount> countsToWheelSpeeds(
  const std::array<int32_t, kWheelCount> & delta_counts,
  double dt,
  const ChassisGeometry & geom)
{
  std::array<double, kWheelCount> out{};
  if (dt <= 1e-6) {
    return out;
  }
  const double m_per_count = geom.wheel_circumference() / geom.counts_per_rev();
  for (int i = 0; i < kWheelCount; ++i) {
    out[i] = static_cast<double>(delta_counts[i]) * m_per_count / dt;
  }
  return out;
}

BodyVelocity wheelSpeedsToBody(
  const std::array<double, kWheelCount> & w,
  const ChassisGeometry & geom)
{
  BodyVelocity body;
  // 与 encoder.c:96-98 等价
  body.vx = (w[kWheelFL] + w[kWheelRR] + w[kWheelRL] + w[kWheelFR]) * 0.25;
  body.vy = (-w[kWheelFL] - w[kWheelRR] + w[kWheelRL] + w[kWheelFR]) * 0.25;
  body.wz = (-w[kWheelFL] + w[kWheelRR] - w[kWheelRL] + w[kWheelFR]) /
            (4.0 * geom.wheel_base_sum_m);
  return body;
}

std::array<double, kWheelCount> bodyToWheelSpeeds(
  const BodyVelocity & body,
  const ChassisGeometry & geom)
{
  const double k = geom.wheel_base_sum_m * body.wz;
  std::array<double, kWheelCount> w{};
  w[kWheelFL] = body.vx - body.vy - k;
  w[kWheelRR] = body.vx - body.vy + k;
  w[kWheelRL] = body.vx + body.vy - k;
  w[kWheelFR] = body.vx + body.vy + k;
  return w;
}

}  // namespace seedling_chassis
