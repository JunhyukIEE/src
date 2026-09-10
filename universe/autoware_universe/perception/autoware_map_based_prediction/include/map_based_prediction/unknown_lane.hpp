// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#ifndef MAP_BASED_PREDICTION_UNKNOWN_LANE_HPP_
#define MAP_BASED_PREDICTION_UNKNOWN_LANE_HPP_
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <boost/geometry.hpp>
#include <cmath>
#include <optional>
#include <deque>
#include <vector>

namespace autoware::map_based_prediction
{
using UnknownMotionHistory = std::deque<std::pair<double, geometry_msgs::msg::Point>>;
// Recover a cold tracker only from consistent measured displacement, never from an assumed NPC speed.
inline void recover_unknown_velocity(
  autoware_perception_msgs::msg::TrackedObject & object, UnknownMotionHistory & history,
  const double time)
{
  const auto & pose = object.kinematics.pose_with_covariance.pose;
  if (!std::isfinite(time) || !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y)) return;
  if (!history.empty() && (time <= history.back().first || time - history.back().first > 0.5))
    history.clear();
  history.emplace_back(time, pose.position);
  while (history.size() > 1 && time - history.front().first > 0.6) history.pop_front();
  if (history.size() < 4 || time - history.front().first < 0.29) return;
  const double dt = time - history.front().first;
  const double vx = (pose.position.x - history.front().second.x) / dt;
  const double vy = (pose.position.y - history.front().second.y) / dt;
  const double speed = std::hypot(vx, vy);
  if (speed < 1.0 || speed > 15.0) return;
  for (size_t i = 1; i < history.size(); ++i) {
    const double interval = history[i].first - history[i-1].first;
    if (interval < 0.02 || std::hypot(
        (history[i].second.x - history[i-1].second.x) / interval - vx,
        (history[i].second.y - history[i-1].second.y) / interval - vy) > 2.0) return;
  }
  auto & velocity = object.kinematics.twist_with_covariance.twist.linear;
  if (std::hypot(velocity.x, velocity.y) >= speed * 0.5) return;
  const double yaw = tf2::getYaw(pose.orientation);
  if (!std::isfinite(yaw)) return;
  velocity.x = vx * std::cos(yaw) + vy * std::sin(yaw);
  velocity.y = -vx * std::sin(yaw) + vy * std::cos(yaw);
  auto & cov = object.kinematics.twist_with_covariance.covariance;
  cov[0] = std::max(4.0, cov[0]);
  cov[7] = std::max(4.0, cov[7]);
}

// Internal prediction candidate only: never change the published semantic class.
inline std::optional<autoware_perception_msgs::msg::TrackedObject> unknown_lane_candidate(
  const autoware_perception_msgs::msg::TrackedObject & object,
  const std::vector<double> & bounds)
{
  if (bounds.size() != 4) return std::nullopt;
  for (const auto b : bounds) if (!std::isfinite(b)) return std::nullopt;
  if (bounds[0] >= bounds[1] || bounds[2] >= bounds[3]) return std::nullopt;
  const auto & pose = object.kinematics.pose_with_covariance.pose;
  const auto & velocity = object.kinematics.twist_with_covariance.twist.linear;
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(velocity.x) || !std::isfinite(velocity.y) ||
      pose.position.x < bounds[0] || pose.position.x > bounds[1] ||
      pose.position.y < bounds[2] || pose.position.y > bounds[3]) return std::nullopt;
  const double speed = std::hypot(velocity.x, velocity.y);
  if (speed < 1.0 || speed > 20.0) return std::nullopt;
  using Shape = autoware_perception_msgs::msg::Shape;
  if (object.shape.type != Shape::POLYGON && object.shape.type != Shape::BOUNDING_BOX)
    return std::nullopt;
  for (const auto & v : object.shape.footprint.points)
    if (!std::isfinite(v.x) || !std::isfinite(v.y)) return std::nullopt;
  geometry_msgs::msg::Pose origin;
  origin.orientation.w = 1.0;
  const auto polygon = autoware_utils::to_polygon2d(origin, object.shape);
  if (polygon.outer().size() < 4 || !std::isfinite(boost::geometry::area(polygon)) ||
      std::abs(boost::geometry::area(polygon)) < 0.5) return std::nullopt;
  autoware_utils::Box2d box;
  boost::geometry::envelope(polygon, box);
  const double x = box.max_corner().x() - box.min_corner().x();
  const double y = box.max_corner().y() - box.min_corner().y();
  if (std::max(x, y) > 12.0 || std::max(x, y) < 1.5 || std::min(x, y) < 0.5)
    return std::nullopt;
  auto aligned = object;
  const double delta = std::atan2(velocity.y, velocity.x);
  const double yaw = tf2::getYaw(pose.orientation);
  if (!std::isfinite(yaw)) return std::nullopt;
  aligned.kinematics.pose_with_covariance.pose.orientation =
    autoware_utils::create_quaternion_from_yaw(yaw + delta);
  aligned.kinematics.orientation_availability =
    autoware_perception_msgs::msg::TrackedObjectKinematics::AVAILABLE;
  aligned.kinematics.twist_with_covariance.twist.linear.x = speed;
  aligned.kinematics.twist_with_covariance.twist.linear.y = 0.0;
  // This temporary geometry is used only to generate paths. Output geometry is restored.
  aligned.shape.footprint = autoware_utils::rotate_polygon(object.shape.footprint, -delta);
  return aligned;
}
}  // namespace autoware::map_based_prediction
#endif
