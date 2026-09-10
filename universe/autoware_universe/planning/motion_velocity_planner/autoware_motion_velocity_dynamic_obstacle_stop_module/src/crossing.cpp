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
#include "crossing.hpp"
#include <autoware/motion_velocity_planner_common/roundabout_gap.hpp>

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoware::motion_velocity_planner::dynamic_obstacle_stop
{
CrossingResult find_predicted_crossings(
  const EgoData & ego, const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects,
  const PlannerParam & p)
{
  CrossingResult result;
  if (ego.trajectory.size() < 2 || ego.trajectory_footprints.empty()) return result;
  if (!std::isfinite(ego.prediction_age) || ego.prediction_age > p.time_horizon ||
      !std::isfinite(ego.velocity)) return result;
  double nearest_approach = p.approach_distance;
  for (const auto & object : objects) {
    if (roundabout_gap::is_entry_crossing_vehicle(object, ego.pose)) {
      if (roundabout_gap::occupies_entry_gate(object)) {
        result.collisions.push_back(
          {object.kinematics.initial_pose_with_covariance.pose.position,
           autoware_utils::to_hex_string(object.object_id)});
      } else {
        result.cleared_objects.push_back(autoware_utils::to_hex_string(object.object_id));
      }
      continue;
    }
    geometry_msgs::msg::Point conflict;
    const auto gap = roundabout_gap::clear(object, ego.pose, ego.velocity, ego.prediction_age,
      ego.trajectory, ego.trajectory_footprints, &conflict);
    if (gap && *gap) {
      result.cleared_objects.push_back(autoware_utils::to_hex_string(object.object_id));
      continue;
    }
    if (gap) {
      result.collisions.push_back({conflict, autoware_utils::to_hex_string(object.object_id)});
      continue;
    }
    const auto & shape = object.shape;
    const auto & velocity = object.kinematics.initial_twist_with_covariance.twist.linear;
    if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y)) continue;
    using Shape = autoware_perception_msgs::msg::Shape;
    if (shape.type == Shape::POLYGON) {
      if (shape.footprint.points.size() < 3) continue;
      if (std::any_of(shape.footprint.points.begin(), shape.footprint.points.end(),
          [](const auto & v) { return !std::isfinite(v.x) || !std::isfinite(v.y); })) continue;
    } else if (shape.type == Shape::BOUNDING_BOX || shape.type == Shape::CYLINDER) {
      if (!std::isfinite(shape.dimensions.x) || shape.dimensions.x <= 0.0) continue;
      if (shape.type == Shape::BOUNDING_BOX &&
          (!std::isfinite(shape.dimensions.y) || shape.dimensions.y <= 0.0)) continue;
    } else {
      continue;
    }
    geometry_msgs::msg::Pose origin;
    origin.orientation.w = 1.0;
    const auto local_polygon = autoware_utils::to_polygon2d(origin, shape);
    if (std::abs(boost::geometry::area(local_polygon)) < 1e-6) continue;
    // UNKNOWN is allowed: a valid tracked footprint must not disappear at a class transition.
    auto paths = object.kinematics.predicted_paths;
    paths.erase(
      std::remove_if(
        paths.begin(), paths.end(),
        [](const auto & path) { return path.confidence < 0.1 || path.path.size() < 2; }),
      paths.end());
    if (paths.empty()) {
      autoware_perception_msgs::msg::PredictedPath path;
      path.confidence = 1.0;
      path.time_step.nanosec = 200000000;
      const auto initial = object.kinematics.initial_pose_with_covariance.pose;
      const double yaw = tf2::getYaw(initial.orientation);
      for (double t = 0.0; t <= p.time_horizon + std::max(0.0, ego.prediction_age); t += 0.2) {
        auto pose = initial;
        pose.position.x += t * (velocity.x * std::cos(yaw) - velocity.y * std::sin(yaw));
        pose.position.y += t * (velocity.x * std::sin(yaw) + velocity.y * std::cos(yaw));
        path.path.push_back(pose);
      }
      paths.push_back(path);
    }
    std::optional<geometry_msgs::msg::Point> object_stop;
    double nearest_stop = std::numeric_limits<double>::max();
    for (const auto & path : paths) {
      double npc_arc = 0.0;
      const double step = path.time_step.sec + path.time_step.nanosec * 1e-9;
      if (!std::isfinite(step) || step <= 0.0) continue;
      // Each swept segment is tested, including polygon containment (not just edge crossings).
      for (size_t k = 1; k < path.path.size(); ++k) {
        const double speed = std::hypot(velocity.x, velocity.y);
        const bool constant_npc = gap.has_value();
        const double t0 = (constant_npc ? npc_arc / speed : (k - 1) * step) - ego.prediction_age;
        npc_arc += std::hypot(path.path[k].position.x - path.path[k-1].position.x,
          path.path[k].position.y - path.path[k-1].position.y);
        const double t1 = (constant_npc ? npc_arc / speed : k * step) - ego.prediction_age;
        if (t1 < 0.0) continue;
        if (t0 > p.time_horizon) break;
        const auto & a = path.path[k - 1];
        const auto & b = path.path[k];
        if (
          !std::isfinite(a.position.x) || !std::isfinite(a.position.y) ||
          !std::isfinite(b.position.x) || !std::isfinite(b.position.y) ||
          !std::isfinite(tf2::getYaw(a.orientation)) ||
          !std::isfinite(tf2::getYaw(b.orientation)))
          continue;
        const auto fa = autoware_utils::to_polygon2d(a, shape);
        const auto fb = autoware_utils::to_polygon2d(b, shape);
        autoware_utils::MultiPoint2d corners;
        const double margin = std::max(0.0, p.extra_object_width) / 2;
        for (const auto * polygon : {&fa, &fb}) {
          for (const auto & v : polygon->outer()) {
            for (double dx : {-margin, margin})
              for (double dy : {-margin, margin}) corners.emplace_back(v.x() + dx, v.y() + dy);
          }
        }
        autoware_utils::Polygon2d swept;
        boost::geometry::convex_hull(corners, swept);
        std::vector<BoxIndexPair> candidates;
        ego.rtree.query(boost::geometry::index::intersects(swept), std::back_inserter(candidates));
        for (const auto & candidate : candidates) {
          const auto i = candidate.second;
          const auto & pose = ego.trajectory[i].pose;
          // Same-direction following remains the responsibility of obstacle_cruise.
          const double dx = b.position.x - a.position.x;
          const double dy = b.position.y - a.position.y;
          const double angle = std::abs(autoware_utils::normalize_radian(
            std::atan2(dy, dx) - tf2::getYaw(pose.orientation)));
          // A stationary footprint still blocks the path, irrespective of its box orientation.
          if (std::hypot(dx, dy) > 1e-3 && angle < p.crossing_min_angle) continue;
          if (!boost::geometry::intersects(swept, ego.trajectory_footprints[i])) continue;
          const double distance = autoware::motion_utils::calcSignedArcLength(
            ego.trajectory, ego.pose.position, pose.position);
          if (distance < 0.0) continue;
          // Use earliest physically possible arrival, including departure from standstill.
          const double v = std::max(0.0, ego.velocity);
          const double acc = std::max(0.1, p.departure_acceleration);
          const double arrival = constant_npc ? roundabout_gap::arrival(distance,v) :
            2.0 * distance / (std::sqrt(v * v + 2 * acc * distance) + v + 1e-6);
          const double margin = constant_npc ? roundabout_gap::time_margin : p.crossing_time_margin;
          if (t1 + margin < arrival) continue;  // cleared before we can arrive
          if (distance < nearest_approach) {
            nearest_approach = distance;
            result.approach_point = pose.position;
          }
          // Both vehicle extents are already included in the swept polygon test.
          // Adding vehicle lengths as another time buffer double-counts occupancy and
          // joins separate cars into a continuous stop at a roundabout.
          if (t0 > arrival + margin) continue;
          if (distance < nearest_stop) {
            nearest_stop = distance;
            object_stop = pose.position;
          }
        }
      }
    }
    if (object_stop)
      result.collisions.push_back({*object_stop, autoware_utils::to_hex_string(object.object_id)});
  }
  return result;
}
}  // namespace autoware::motion_velocity_planner::dynamic_obstacle_stop
