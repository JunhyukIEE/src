// Copyright 2026 FML
// Licensed under the Apache License, Version 2.0.
#ifndef KCITY_SPEED_LIMIT_HPP_
#define KCITY_SPEED_LIMIT_HPP_

#include <algorithm>
#include <cmath>

namespace tier4_adapi_rviz_plugins::kcity
{
// Supplied UTM zone 52N coordinates, expressed in the MGRS 52SCG map frame.
inline constexpr double speed_zone_start_x = 302657.074 - 300000.0;
inline constexpr double speed_zone_start_y = 4124401.495 - 4100000.0;
inline constexpr double speed_zone_end_x = 302665.214 - 300000.0;
inline constexpr double speed_zone_end_y = 4123779.252 - 4100000.0;
inline constexpr double speed_limit_mps = 60.0 / 3.6;

inline bool in_unlimited_speed_zone(double x, double y)
{
  const double dx = speed_zone_end_x - speed_zone_start_x;
  const double dy = speed_zone_end_y - speed_zone_start_y;
  const double px = x - speed_zone_start_x;
  const double py = y - speed_zone_start_y;
  const double length_squared = dx * dx + dy * dy;
  const double fraction = (px * dx + py * dy) / length_squared;
  const double lateral = (px * dy - py * dx) / std::sqrt(length_squared);
  // shortcut: straight-section corridor covering adjacent lanes, not a lanelet speed-limit map.
  // Replace the corridor if the course bends or the unrestricted section changes.
  return fraction >= 0.0 && fraction < 1.0 && std::abs(lateral) <= 20.0;
}

struct SpeedLimitStatus
{
  bool valid{false}, unlimited{false}, warning{false};
  double kmh{0.0}, peak_kmh{0.0};
};

class SpeedLimitWatch
{
public:
  SpeedLimitStatus update(double x, double y, double speed_mps, double now, bool fresh)
  {
    SpeedLimitStatus status;
    status.valid = fresh && std::isfinite(x) && std::isfinite(y) && std::isfinite(speed_mps) &&
                   std::isfinite(now);
    if (!status.valid) {
      warning_until_ = 0.0;
      peak_kmh_ = 0.0;
      return status;
    }
    status.kmh = std::abs(speed_mps) * 3.6;
    status.unlimited = in_unlimited_speed_zone(x, y);
    if (status.unlimited) {
      warning_until_ = 0.0;
      peak_kmh_ = 0.0;
      return status;
    }
    if (std::abs(speed_mps) > speed_limit_mps) {
      peak_kmh_ = now < warning_until_ ? std::max(peak_kmh_, status.kmh) : status.kmh;
      warning_until_ = now + 2.0;  // Keep even a single-sample exceedance visible.
    }
    status.warning = now < warning_until_;
    status.peak_kmh = status.warning ? peak_kmh_ : 0.0;
    return status;
  }

private:
  double warning_until_{0.0}, peak_kmh_{0.0};
};
}  // namespace tier4_adapi_rviz_plugins::kcity
#endif  // KCITY_SPEED_LIMIT_HPP_
