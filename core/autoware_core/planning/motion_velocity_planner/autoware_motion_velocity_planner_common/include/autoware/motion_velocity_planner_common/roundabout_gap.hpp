// Copyright 2026 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef AUTOWARE_MOTION_VELOCITY_PLANNER_COMMON_ROUNDABOUT_GAP_HPP_
#define AUTOWARE_MOTION_VELOCITY_PLANNER_COMMON_ROUNDABOUT_GAP_HPP_
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <boost/geometry.hpp>
#include <algorithm>
#include <cmath>
#include <optional>
#include <limits>

namespace autoware::motion_velocity_planner::roundabout_gap
{
// MORAI K-City competition: fixed synthetic stop line supplied by the user.
constexpr double entry_stop_line_start_x = 2500.00;
constexpr double entry_stop_line_start_y = 24467.00;
constexpr double entry_stop_line_end_x = 2493.73;
constexpr double entry_stop_line_end_y = 24467.35;
constexpr double entry_braking_start_y = 24408.0;
inline const autoware_utils_geometry::Polygon2d & entry_stop_approach_polygon()
{
  // Arm braking upstream; this does not change the fixed stop line.
  static const autoware_utils_geometry::Polygon2d polygon{
    {{2493, entry_braking_start_y}, {2493, entry_stop_line_end_y},
     {2510, entry_stop_line_end_y}, {2510, entry_braking_start_y}, {2493, entry_braking_start_y}}, {}};
  return polygon;
}
inline const autoware_utils_geometry::LineString2d & entry_stop_line()
{
  static const autoware_utils_geometry::LineString2d line{
    {{entry_stop_line_start_x, entry_stop_line_start_y},
     {entry_stop_line_end_x, entry_stop_line_end_y}}};
  return line;
}
inline double entry_stop_line_y_at(const double x)
{
  const double slope = (entry_stop_line_end_y - entry_stop_line_start_y) /
    (entry_stop_line_end_x - entry_stop_line_start_x);
  return entry_stop_line_start_y + slope * (x - entry_stop_line_start_x);
}
inline std::optional<geometry_msgs::msg::Point> entry_stop_line_intersection(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const geometry_msgs::msg::Point & ego_position)
{
  if (trajectory.size() < 2) return std::nullopt;
  autoware_utils_geometry::LineString2d trajectory_line;
  for (const auto & point : trajectory) {
    trajectory_line.emplace_back(point.pose.position.x, point.pose.position.y);
  }
  autoware_utils_geometry::MultiPoint2d intersections;
  boost::geometry::intersection(trajectory_line, entry_stop_line(), intersections);
  std::optional<geometry_msgs::msg::Point> nearest;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (const auto & intersection : intersections) {
    geometry_msgs::msg::Point point;
    point.x = intersection.x();
    point.y = intersection.y();
    const double distance = autoware::motion_utils::calcSignedArcLength(
      trajectory, ego_position, point);
    // Keep the fixed line after passing it too: the caller must distinguish
    // "passed the line" from "the route does not cross the line".
    if (std::abs(distance) < nearest_distance) {
      nearest = point;
      nearest_distance = std::abs(distance);
    }
  }
  return nearest;
}
inline bool in_entry_stop_arm_region(const geometry_msgs::msg::Point & p)
{
  const autoware_utils_geometry::Point2d point{p.x, p.y};
  return boost::geometry::covered_by(point, entry_stop_approach_polygon()) ||
         (p.x >= 2493 && p.x <= 2510 && p.y > entry_stop_line_y_at(p.x) && p.y < 24478);
}
inline bool has_reached_entry_waiting_point(const geometry_msgs::msg::Point & p)
{
  return p.y >= entry_stop_line_y_at(p.x) - 0.5;
}
inline bool has_passed_entry_waiting_point(const double signed_distance)
{
  return std::isfinite(signed_distance) && signed_distance <= 0.0;
}
// MORAI K-City competition only. Calibration values, not a general road policy.
inline bool in_region(const geometry_msgs::msg::Point & p)
{
  // Entry approach only. After entering, normal following takes over.
  return p.x >= 2493 && p.x <= 2503 && p.y >= 24445 && p.y < 24468;
}
inline bool in_entry_commit_region(const geometry_msgs::msg::Point & p)
{
  // Keep an approved launch through the physical entry, then return to normal planning.
  return p.x >= 2493 && p.x <= 2510 && p.y >= 24445 && p.y < 24478;
}
inline autoware_utils_geometry::Polygon2d entry_zone()
{
  return {{{2493,24468},{2493,24476},{2503,24476},{2503,24468},{2493,24468}}, {}};
}
inline autoware_utils_geometry::Polygon2d entry_gate()
{
  // MORAI K-City competition: NPC rear must clear this map-coordinate "door" before launch.
  return {{{2495.10,24468.72},{2495.10,24473.20},{2496.22,24473.20},{2496.22,24468.72},{2495.10,24468.72}}, {}};
}
constexpr double acceleration = 6.0;  // competition roundabout departure acceleration, m/s^2
constexpr double target_speed = 15.0 / 3.6;  // 15 km/h
inline bool is_entry_crossing_vehicle(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const geometry_msgs::msg::Pose & ego)
{
  if (!in_entry_commit_region(ego.position)) return false;
  const auto & v = object.kinematics.initial_twist_with_covariance.twist.linear;
  const double speed = std::hypot(v.x, v.y);
  if (!std::isfinite(speed) || speed < 1.0 || speed > 20.0) return false;
  const auto & p = object.kinematics.initial_pose_with_covariance.pose.position;
  const double yaw = tf2::getYaw(ego.orientation);
  const double object_yaw = tf2::getYaw(object.kinematics.initial_pose_with_covariance.pose.orientation);
  if (!std::isfinite(yaw) || !std::isfinite(object_yaw) || !std::isfinite(p.x) || !std::isfinite(p.y)) return false;
  const double dx = p.x - ego.position.x;
  const double dy = p.y - ego.position.y;
  const double longitudinal = std::cos(yaw) * dx + std::sin(yaw) * dy;
  // PredictedObject twist is in the object's frame, so compare its map velocity with ego yaw.
  const double vx = std::cos(object_yaw) * v.x - std::sin(object_yaw) * v.y;
  const double vy = std::sin(object_yaw) * v.x + std::cos(object_yaw) * v.y;
  const double lateral_speed = -std::sin(yaw) * vx + std::cos(yaw) * vy;
  // Only traffic crossing the entry in front of the bumper is delegated.
  return longitudinal >= 0.0 && longitudinal <= 35.0 &&
         std::abs(lateral_speed) >= 0.5 * speed;
}
inline bool has_valid_shape(const autoware_perception_msgs::msg::PredictedObject & object)
{
  using Shape = autoware_perception_msgs::msg::Shape;
  if (object.shape.type == Shape::POLYGON && object.shape.footprint.points.size() < 3) return false;
  if (object.shape.type == Shape::BOUNDING_BOX &&
      (!(object.shape.dimensions.x > 0.0) || !(object.shape.dimensions.y > 0.0))) return false;
  if (object.shape.type == Shape::CYLINDER && !(object.shape.dimensions.x > 0.0)) return false;
  if (object.shape.type != Shape::POLYGON && object.shape.type != Shape::BOUNDING_BOX &&
      object.shape.type != Shape::CYLINDER) return false;
  return true;
}
inline bool occupies_entry_zone(const autoware_perception_msgs::msg::PredictedObject & object)
{
  if (!has_valid_shape(object)) return false;
  return boost::geometry::intersects(
    autoware_utils_geometry::to_polygon2d(object.kinematics.initial_pose_with_covariance.pose, object.shape),
    entry_zone());
}
inline bool occupies_entry_gate(const autoware_perception_msgs::msg::PredictedObject & object)
{
  if (!has_valid_shape(object)) return false;
  return boost::geometry::intersects(
    autoware_utils_geometry::to_polygon2d(object.kinematics.initial_pose_with_covariance.pose, object.shape),
    entry_gate());
}
inline bool apply_entry_launch_profile(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const geometry_msgs::msg::Point & ego_position)
{
  if (trajectory.empty() || !in_entry_commit_region(ego_position)) return false;

  size_t nearest_idx = 0;
  double nearest_distance_sq = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < trajectory.size(); ++i) {
    const auto & p = trajectory[i].pose.position;
    const double distance_sq = (p.x - ego_position.x) * (p.x - ego_position.x) +
      (p.y - ego_position.y) * (p.y - ego_position.y);
    if (distance_sq < nearest_distance_sq) {
      nearest_distance_sq = distance_sq;
      nearest_idx = i;
    }
  }

  double distance = 0.0;
  bool applied = false;
  for (size_t i = nearest_idx; i < trajectory.size(); ++i) {
    if (i > nearest_idx) {
      const auto & prev = trajectory[i - 1].pose.position;
      const auto & curr = trajectory[i].pose.position;
      distance += std::hypot(curr.x - prev.x, curr.y - prev.y);
    }
    if (!in_entry_commit_region(trajectory[i].pose.position)) break;

    // The controller tracks the point at ego, so a distance ramp would reset its target to 0.5 m/s
    // on every cycle. The controller applies the acceleration limit to this immediate target.
    trajectory[i].longitudinal_velocity_mps = target_speed;
    trajectory[i].acceleration_mps2 = acceleration;
    applied = true;
  }
  return applied;
}
constexpr double start_delay = 0.3;   // s
constexpr double time_margin = 0.5;   // s, each side of the occupancy window
inline double arrival(double distance, double speed)
{
  distance = std::max(0.0, distance);
  speed = std::max(0.0, speed);
  const double cap = std::max(target_speed, speed);
  const double accelerate_distance = (cap * cap - speed * speed) / (2 * acceleration);
  const double delay = speed < 0.5 ? start_delay : 0.0;
  if (distance <= accelerate_distance)
    return delay + 2 * distance / (std::sqrt(speed * speed + 2 * acceleration * distance) + speed + 1e-9);
  return delay + (cap - speed) / acceleration + (distance - accelerate_distance) / cap;
}

// nullopt: not applicable/insufficient evidence; caller retains its normal stop logic.
inline std::optional<bool> clear(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const geometry_msgs::msg::Pose & ego, const double ego_speed, const double age,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const std::vector<autoware_utils_geometry::Polygon2d> & ego_polygons,
  geometry_msgs::msg::Point * conflict = nullptr)
{
  if (!in_region(ego.position) || age < 0 || age > 0.5 || !std::isfinite(age) ||
      !std::isfinite(ego_speed) || trajectory.size() != ego_polygons.size()) return std::nullopt;
  if (trajectory.size() < 2) return std::nullopt;
  const auto zone = entry_zone();
  const auto & v = object.kinematics.initial_twist_with_covariance.twist.linear;
  const double speed = std::hypot(v.x,v.y);
  if (!std::isfinite(speed) || speed < 1.0 || speed > 20.0) return std::nullopt;
  auto motion_pose = object.kinematics.initial_pose_with_covariance.pose;
  const double yaw = tf2::getYaw(motion_pose.orientation);
  if (!std::isfinite(yaw) || trajectory.size() < 2) return std::nullopt;
  motion_pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw + std::atan2(v.y,v.x));
  const double angle = std::abs(autoware::motion_utils::calc_diff_angle_against_trajectory(trajectory,motion_pose));
  if (angle < 0.523599 || angle > 2.617994) return std::nullopt;
  using Shape = autoware_perception_msgs::msg::Shape;
  if (object.shape.type == Shape::POLYGON) {
    if (object.shape.footprint.points.size() < 3) return std::nullopt;
    for (const auto & p : object.shape.footprint.points)
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) return std::nullopt;
  } else if (object.shape.type == Shape::BOUNDING_BOX || object.shape.type == Shape::CYLINDER) {
    if (!(object.shape.dimensions.x > 0) || !std::isfinite(object.shape.dimensions.x) ||
        (object.shape.type == Shape::BOUNDING_BOX &&
          (!(object.shape.dimensions.y > 0) || !std::isfinite(object.shape.dimensions.y)))) return std::nullopt;
  } else return std::nullopt;
  double exit_distance = -1.0;
  bool beyond_zone = false;
  std::vector<double> distances(trajectory.size(), -1.0);
  for (size_t i=0; i<trajectory.size(); ++i) {
    const double distance = autoware::motion_utils::calcSignedArcLength(
      trajectory, ego.position, trajectory[i].pose.position);
    if (distance < 0) continue;
    if (boost::geometry::intersects(ego_polygons[i], zone)) {
      distances[i] = distance;
      exit_distance = distance;
      beyond_zone = false;
    } else if (exit_distance >= 0) {
      beyond_zone = true;
      break;
    }
  }
  // A truncated stopped trajectory cannot establish the time to clear the zone.
  if (!beyond_zone) return std::nullopt;
  const double exit_time = arrival(exit_distance + 1.1, ego_speed); // rear overhang allowance
  // The zone limits where this exception applies, not an area that must remain empty.
  // Compare occupancy of each ego footprint at its arrival time instead.
  bool tested = false;
  for (const auto & path : object.kinematics.predicted_paths) {
    if (!std::isfinite(path.confidence) || path.confidence < 0.1 || path.path.size() < 2) continue;
    double arc = 0;
    for (size_t k=1; k<path.path.size(); ++k) {
      const auto & a=path.path[k-1]; const auto & b=path.path[k];
      if (!std::isfinite(a.position.x) || !std::isfinite(a.position.y) ||
          !std::isfinite(b.position.x) || !std::isfinite(b.position.y) ||
          !std::isfinite(tf2::getYaw(a.orientation)) || !std::isfinite(tf2::getYaw(b.orientation))) return std::nullopt;
      const double t0=arc/speed-age;
      arc+=std::hypot(b.position.x-a.position.x,b.position.y-a.position.y);
      const double t1=arc/speed-age;
      if (t1 < 0) continue;
      tested = true;
      autoware_utils_geometry::MultiPoint2d corners;
      for (const auto & pose : {a,b}) {
        const auto polygon=autoware_utils_geometry::to_polygon2d(pose,object.shape);
        for (const auto & point : polygon.outer()) corners.push_back(point);
      }
      autoware_utils_geometry::Polygon2d swept;
      boost::geometry::convex_hull(corners,swept);
      if (std::abs(boost::geometry::area(swept)) < 1e-6) return std::nullopt;
      if (!boost::geometry::intersects(swept,zone)) continue;
      for (size_t i=0; i<distances.size(); ++i) {
        if (distances[i] < 0) continue;
        const double enter = arrival(distances[i], ego_speed);
        const double leave = arrival(distances[i] + 1.1, ego_speed);
        if (t0 > leave + time_margin || t1 + time_margin < enter) continue;
        if (boost::geometry::intersects(swept,ego_polygons[i])) {
          if (conflict) *conflict = trajectory[i].pose.position;
          return false;
        }
      }
    }
    if (arc / speed - age < exit_time + time_margin) return std::nullopt;
  }
  return tested ? std::optional<bool>(true) : std::nullopt;
}
}  // namespace autoware::motion_velocity_planner::roundabout_gap
#endif
