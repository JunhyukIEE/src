// Copyright 2026 FML
// Licensed under the Apache License, Version 2.0.
#ifndef CHECKPOINT_PROGRESS_HPP_
#define CHECKPOINT_PROGRESS_HPP_

#include "kcity_course.hpp"

#include <algorithm>

namespace tier4_adapi_rviz_plugins::kcity
{
enum class CheckpointResult { Pending, Success, Missed };

// Display-only scoring; independent of the mission planner's waypoint advancement.
struct CheckpointProgress
{
  std::array<CheckpointResult, course.size()> results{};
  std::size_t next{1};
  bool stopped_after_end{false};

  void reset() { *this = CheckpointProgress{}; }
  bool finished() const { return results[0] != CheckpointResult::Pending; }
  std::size_t successes() const
  {
    return std::count(results.begin(), results.end(), CheckpointResult::Success);
  }

  bool update(
    const geometry_msgs::msg::Pose & previous, const geometry_msgs::msg::Pose & current,
    double speed)
  {
    if (
      !std::isfinite(previous.position.x) || !std::isfinite(previous.position.y) ||
      !std::isfinite(current.position.x) || !std::isfinite(current.position.y) ||
      !std::isfinite(speed) || distance(previous, current) > 30.0)
      return false;
    if (finished()) {
      if (std::abs(speed) < 0.1) stopped_after_end = true;
      if (stopped_after_end && std::abs(speed) > 0.5) {
        reset();
        return true;
      }
      return false;
    }

    // Interpolate only fresh adjacent poses so fast passes between samples are not lost.
    const auto segment_distance = [&](const geometry_msgs::msg::Pose & target) {
      const double dx = current.position.x - previous.position.x;
      const double dy = current.position.y - previous.position.y;
      const double length_squared = dx * dx + dy * dy;
      const double t = length_squared == 0.0 ? 0.0
                                             : std::clamp(
                                                 ((target.position.x - previous.position.x) * dx +
                                                  (target.position.y - previous.position.y) * dy) /
                                                   length_squared,
                                                 0.0, 1.0);
      return std::hypot(
        previous.position.x + t * dx - target.position.x,
        previous.position.y + t * dy - target.position.y);
    };
    bool changed = false;
    for (auto i = next; i < course.size(); ++i) {
      if (segment_distance(pose(i)) > 3.0 + 1e-9) continue;
      // Reaching a later checkpoint settles earlier, still unvisited checkpoints as missed.
      for (auto skipped = next; skipped < i; ++skipped) results[skipped] = CheckpointResult::Missed;
      results[i] = CheckpointResult::Success;
      next = i + 1;
      changed = true;
    }
    const auto index = next == course.size() ? 0 : next;
    const auto target = pose(index);
    if (index == 0 && segment_distance(target) <= 3.0 + 1e-9) {
      results[0] = CheckpointResult::Success;
      stopped_after_end = std::abs(speed) < 0.1;
      return true;
    }
    const double yaw = course[index].yaw_deg * std::acos(-1.0) / 180.0;
    const auto along = [&](const geometry_msgs::msg::Pose & p) {
      return (p.position.x - target.position.x) * std::cos(yaw) +
             (p.position.y - target.position.y) * std::sin(yaw);
    };
    // Resolve a near miss only after moving beyond the entire 3 m acceptance disk.
    if (along(previous) <= 3.0 && along(current) > 3.0 && segment_distance(target) <= 20.0) {
      results[index] = CheckpointResult::Missed;
      if (index != 0) ++next;
      return true;
    }
    return changed;
  }
};
}  // namespace tier4_adapi_rviz_plugins::kcity
#endif  // CHECKPOINT_PROGRESS_HPP_
