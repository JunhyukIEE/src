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

// Offline only: MORAI_CROSSING_BAG=/path/to/bag ctest -R test_... . No ROS publishers.
#include "../src/crossing.hpp"
#include "../src/footprint.hpp"
#include "../src/object_stop_decision.hpp"
#include <autoware/motion_velocity_planner_common/roundabout_gap.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <rclcpp/serialization.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <gtest/gtest.h>
#include <chrono>
#include <cstdlib>
#include <iostream>

using namespace autoware::motion_velocity_planner::dynamic_obstacle_stop;
template<class T> T decode(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & message)
{
  rclcpp::SerializedMessage serialized(*message->serialized_data);
  T value;
  rclcpp::Serialization<T>().deserialize_message(&serialized, &value);
  return value;
}

TEST(PredictedCrossingBag, RecordedInputs)
{
  const char * path = std::getenv("MORAI_CROSSING_BAG");
  if (!path) GTEST_SKIP() << "Set MORAI_CROSSING_BAG for offline replay";
  rosbag2_cpp::Reader reader;
  reader.open(path);
  nav_msgs::msg::Odometry odometry;
  autoware_planning_msgs::msg::Trajectory trajectory;
  bool has_odometry = false;
  PlannerParam p;
  p.time_horizon = 5.0;
  p.ego_longitudinal_offset = 3.8;
  p.ego_lateral_offset = 1.0;
  p.extra_object_width = 0.5;
  p.departure_acceleration = 3.0;
  p.add_duration_buffer = 0.1;
  p.remove_duration_buffer = 0.3;
  ObjectStopDecisionMap decisions;
  EntryStopLatch entry_latch;
  size_t retained_braking_frames = 0;
  size_t frames = 0, crossing_frames = 0, stop_frames = 0, transitions = 0;
  size_t polygon_collisions = 0;
  size_t gate_clear = 0, gate_blocked = 0;
  size_t entry_frames = 0, entry_release_frames = 0;
  double release_start = -1.0, longest_release = 0.0;
  bool stopped = false;
  const auto begin = std::chrono::steady_clock::now();
  while (reader.has_next()) {
    const auto message = reader.read_next();
    if (message->topic_name == "/localization/kinematic_state") {
      odometry = decode<nav_msgs::msg::Odometry>(message);
      has_odometry = true;
    } else if (message->topic_name == "/planning/scenario_planning/trajectory") {
      trajectory = decode<autoware_planning_msgs::msg::Trajectory>(message);
    } else if (message->topic_name == "/perception/object_recognition/objects" && has_odometry &&
               trajectory.points.size() > 1) {
      const auto objects = decode<autoware_perception_msgs::msg::PredictedObjects>(message);
      EgoData ego;
      ego.pose = odometry.pose.pose;
      ego.velocity = odometry.twist.twist.linear.x;
      ego.trajectory.assign(trajectory.points.begin(), trajectory.points.end());
      const rclcpp::Time now(message->time_stamp, RCL_ROS_TIME);
      ego.prediction_age = (now - rclcpp::Time(objects.header.stamp)).seconds();
      make_ego_footprint_rtree(ego, p);
      for (const auto & object : objects.objects) {
        const auto gap = autoware::motion_velocity_planner::roundabout_gap::clear(
          object, ego.pose, ego.velocity, ego.prediction_age,
          ego.trajectory, ego.trajectory_footprints);
        if (gap) { gate_clear += *gap; gate_blocked += !*gap; }
      }
      const auto result = find_predicted_crossings(ego, objects.objects, p);
      for (const auto & collision : result.collisions) {
        for (const auto & object : objects.objects) {
          if (collision.object_uuid == autoware_utils::to_hex_string(object.object_id) &&
              object.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
            ++polygon_collisions;
          }
        }
      }
      for (const auto & id : result.cleared_objects) decisions.erase(id);
      update_object_map(decisions, result.collisions, now, ego.trajectory, p);
      const bool collision_stop = find_earliest_collision(decisions, ego).has_value();
      const bool hold = entry_latch.update(
        autoware::motion_velocity_planner::roundabout_gap::in_region(ego.pose.position),
        collision_stop, ego.velocity, ego.prediction_age,
        rclcpp::Time(objects.header.stamp).nanoseconds());
      const bool stop = collision_stop || hold;
      if (hold && !collision_stop && std::abs(ego.velocity) > 0.3) ++retained_braking_frames;
      if (autoware::motion_velocity_planner::roundabout_gap::in_region(ego.pose.position) &&
          std::abs(ego.velocity) < 0.1) {
        ++entry_frames;
        if (!stop) {
          ++entry_release_frames;
          if (release_start < 0) release_start = now.seconds();
          longest_release = std::max(longest_release, now.seconds() - release_start);
        } else { release_start = -1.0; }
      } else { release_start = -1.0; }
      ++frames;
      crossing_frames += !result.collisions.empty();
      stop_frames += stop;
      transitions += stop != stopped;
      stopped = stop;
    }
  }
  EXPECT_GT(frames, 0U);
  EXPECT_GT(crossing_frames, 0U);
  EXPECT_GT(polygon_collisions, 0U);
  std::cout << "offline frames=" << frames << " crossing=" << crossing_frames
            << " polygon_collisions=" << polygon_collisions
            << " gate_clear=" << gate_clear << " gate_blocked=" << gate_blocked
            << " entry_frames=" << entry_frames << " entry_release_frames=" << entry_release_frames
            << " longest_release_seconds=" << longest_release
            << " retained_braking_frames=" << retained_braking_frames
            << " stop=" << stop_frames << " transitions=" << transitions
            << " seconds=" << std::chrono::duration<double>(
                 std::chrono::steady_clock::now() - begin).count() << std::endl;
}
