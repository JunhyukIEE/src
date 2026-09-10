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
#ifndef DYNAMIC_OBSTACLE_STOP_CROSSING_HPP_
#define DYNAMIC_OBSTACLE_STOP_CROSSING_HPP_
#include "types.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace autoware::motion_velocity_planner::dynamic_obstacle_stop
{
// Competition entry state machine: stop once, observe a vehicle at the gate, then launch on clear.
struct EntryStopLatch
{
  bool holding = false;
  bool stopped_once = false;
  bool saw_gate_vehicle = false;
  uint8_t stopped_fresh_frames = 0;
  bool entry_approved = false;
  int64_t last_stamp = 0;
  bool update(bool enabled, bool conflict, double speed, double age, int64_t stamp)
  {
    if (!enabled) {
      holding = false;
      stopped_once = false;
      saw_gate_vehicle = false;
      stopped_fresh_frames = 0;
      entry_approved = false;
      last_stamp = 0;
      return false;
    }
    if (entry_approved) return false;
    const bool fresh = std::isfinite(age) && age >= 0 && age <= 0.5 && stamp > last_stamp;
    last_stamp = std::max(last_stamp, stamp);
    holding = true;  // A gate clear never authorizes movement before the mandatory stop.
    if (!fresh) return true;
    if (std::abs(speed) > 0.3) {
      stopped_fresh_frames = 0;
      return true;
    }
    stopped_fresh_frames = std::min<uint8_t>(3, stopped_fresh_frames + 1);
    if (stopped_fresh_frames < 3) return true;
    stopped_once = true;
    if (conflict) {
      saw_gate_vehicle = true;
      entry_approved = true;
      holding = false;
    } else if (stopped_once && saw_gate_vehicle) {
      entry_approved = true;
      holding = false;
    }
    return holding;
  }
};
struct CrossingResult
{
  std::vector<Collision> collisions;
  std::vector<std::string> cleared_objects;
  std::optional<geometry_msgs::msg::Point> approach_point;
};
CrossingResult find_predicted_crossings(
  const EgoData & ego, const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects,
  const PlannerParam & params);
}  // namespace autoware::motion_velocity_planner::dynamic_obstacle_stop
#endif
