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
#include "../src/crossing.hpp"
#include "../src/footprint.hpp"
#include "../src/object_stop_decision.hpp"

#include <autoware/motion_velocity_planner_common/roundabout_gap.hpp>
#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>

using namespace autoware::motion_velocity_planner::dynamic_obstacle_stop;

TEST(PredictedCrossing, EntryPriorityClearsOtherLongitudinalConstraints)
{
  autoware::motion_velocity_planner::VelocityPlanningResult result;
  result.stop_points.emplace_back();
  result.slowdown_intervals.emplace_back(geometry_msgs::msg::Point{}, geometry_msgs::msg::Point{}, 0.25);
  autoware_internal_planning_msgs::msg::VelocityLimit limit;
  limit.sender = "obstacle_cruise";
  result.velocity_limit = limit;

  autoware::motion_velocity_planner::clear_longitudinal_constraints(result);

  EXPECT_TRUE(result.stop_points.empty());
  EXPECT_TRUE(result.slowdown_intervals.empty());
  EXPECT_FALSE(result.velocity_limit.has_value());
  ASSERT_TRUE(result.velocity_limit_clear_command.has_value());
  EXPECT_EQ(result.velocity_limit_clear_command->sender, "obstacle_cruise");
}

TEST(PredictedCrossing, ApprovedEntryReplacesOnlyCommitRegionStopVelocities)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> trajectory(22);
  for (size_t i = 0; i < trajectory.size(); ++i) {
    trajectory[i].pose.position.x = 2496.0;
    trajectory[i].pose.position.y = 24460.0 + static_cast<double>(i);
  }
  geometry_msgs::msg::Point ego_position;
  ego_position.x = 2496.0;
  ego_position.y = 24462.0;

  EXPECT_TRUE(autoware::motion_velocity_planner::roundabout_gap::apply_entry_launch_profile(
    trajectory, ego_position));
  EXPECT_NEAR(
    trajectory[2].longitudinal_velocity_mps,
    autoware::motion_velocity_planner::roundabout_gap::target_speed, 1e-5);
  EXPECT_NEAR(
    trajectory[10].longitudinal_velocity_mps,
    autoware::motion_velocity_planner::roundabout_gap::target_speed, 1e-5);
  EXPECT_DOUBLE_EQ(trajectory[2].acceleration_mps2, 6.0);
  EXPECT_NEAR(
    trajectory[17].longitudinal_velocity_mps,
    autoware::motion_velocity_planner::roundabout_gap::target_speed, 1e-5);
  EXPECT_DOUBLE_EQ(trajectory[18].longitudinal_velocity_mps, 0.0);
}

TEST(PredictedCrossing, EntryStopArmsUpstreamButCountsOnlyAtWaitingPoint)
{
  const auto & stop_line = autoware::motion_velocity_planner::roundabout_gap::entry_stop_line();
  EXPECT_DOUBLE_EQ(stop_line.front().x(), 2500.0);
  EXPECT_DOUBLE_EQ(stop_line.front().y(), 24467.0);
  EXPECT_DOUBLE_EQ(stop_line.back().x(), 2493.73);
  EXPECT_DOUBLE_EQ(stop_line.back().y(), 24467.35);
  geometry_msgs::msg::Point point;
  point.x = 2496.0;
  point.y = 24408.0;
  EXPECT_TRUE(autoware::motion_velocity_planner::roundabout_gap::in_entry_stop_arm_region(point));
  EXPECT_FALSE(autoware::motion_velocity_planner::roundabout_gap::has_reached_entry_waiting_point(point));
  point.y = 24435.0;
  EXPECT_TRUE(autoware::motion_velocity_planner::roundabout_gap::in_entry_stop_arm_region(point));
  point.y = 24407.99;
  EXPECT_FALSE(autoware::motion_velocity_planner::roundabout_gap::in_entry_stop_arm_region(point));
  point.y = 24467.0;
  EXPECT_TRUE(autoware::motion_velocity_planner::roundabout_gap::has_reached_entry_waiting_point(point));
  point.y = 24478.0;
  EXPECT_FALSE(autoware::motion_velocity_planner::roundabout_gap::in_entry_stop_arm_region(point));
}

TEST(PredictedCrossing, StopPointIsTrajectoryIntersectionWithStopLine)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> trajectory(2);
  trajectory[0].pose.position.x = 2495.0;
  trajectory[0].pose.position.y = 24450.0;
  trajectory[1].pose.position.x = 2497.0;
  trajectory[1].pose.position.y = 24470.0;
  const auto intersection =
    autoware::motion_velocity_planner::roundabout_gap::entry_stop_line_intersection(
      trajectory, trajectory[0].pose.position);
  ASSERT_TRUE(intersection);
  EXPECT_NEAR(intersection->x, 2496.7183188, 1e-6);
  EXPECT_NEAR(intersection->y, 24467.1831879, 1e-6);

  // Crossing the line must still report that same fixed line, so the caller
  // can hold at the current pose instead of misclassifying it as no crossing.
  const auto passed_intersection =
    autoware::motion_velocity_planner::roundabout_gap::entry_stop_line_intersection(
      trajectory, trajectory[1].pose.position);
  ASSERT_TRUE(passed_intersection);
  EXPECT_NEAR(passed_intersection->x, intersection->x, 1e-6);
  EXPECT_NEAR(passed_intersection->y, intersection->y, 1e-6);
}

TEST(PredictedCrossing, PassedStopLineRequiresCurrentPositionHold)
{
  EXPECT_FALSE(autoware::motion_velocity_planner::roundabout_gap::has_passed_entry_waiting_point(0.01));
  EXPECT_TRUE(autoware::motion_velocity_planner::roundabout_gap::has_passed_entry_waiting_point(0.0));
  EXPECT_TRUE(autoware::motion_velocity_planner::roundabout_gap::has_passed_entry_waiting_point(-0.01));
}

TEST(PredictedCrossing, KeepWaitingPointWhileBrakingReleaseOnFreshClearAtCrawl)
{
  EntryStopLatch latch;
  EXPECT_TRUE(latch.update(true, false, 6.0, 0.05, 1)); // Mandatory stop before gate matching.
  EXPECT_TRUE(latch.update(true, true, 6.0, 0.05, 2)); // Gate is irrelevant while braking.
  EXPECT_TRUE(latch.update(true, false, 5.0, 0.05, 3));
  EXPECT_TRUE(latch.update(true, false, 0.0, 0.05, 4)); // First stable-stop frame.
  EXPECT_TRUE(latch.update(true, true, 0.2, 0.05, 5));  // Second stable-stop frame.
  EXPECT_FALSE(latch.update(true, true, 0.2, 0.05, 6));  // Stop confirmed; gate occupied launches immediately.
  EXPECT_TRUE(latch.entry_approved);
  EXPECT_FALSE(latch.update(true, true, 0.2, 0.05, 9)); // Approval prevents a mid-entry re-stop.
  EXPECT_FALSE(latch.update(true, false, 1.5, 0.05, 10));
  EXPECT_FALSE(latch.update(false, true, 0.2, 0.05, 9));
  EXPECT_FALSE(latch.entry_approved);
  EXPECT_TRUE(latch.update(true, false, 4.0, 0.05, 11)); // Next traversal starts with its stop.
}

TEST(PredictedCrossing, FreshClearanceReleasesButMissingObservationDoesNot)
{
  EgoData ego;
  ego.pose.position.x = 2496;
  ego.pose.position.y = 24462;
  ego.pose.orientation = autoware_utils::create_quaternion_from_yaw(M_PI_2);
  PlannerParam p;
  p.ego_longitudinal_offset = 3.8;
  p.ego_lateral_offset = 1.0;
  for (int i = 0; i < 35; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose = ego.pose;
    point.pose.position.y += i;
    ego.trajectory.push_back(point);
  }
  make_ego_footprint_rtree(ego, p);
  autoware_perception_msgs::msg::PredictedObject object;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 4;
  object.shape.dimensions.y = 2;
  object.kinematics.initial_twist_with_covariance.twist.linear.x = 5;
  autoware_perception_msgs::msg::PredictedPath path;
  path.confidence = 1;
  path.time_step.nanosec = 500000000;
  for (int i = 0; i <= 20; ++i) {
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1;
    pose.position.x = 2508 + 2.5 * i;
    pose.position.y = 24471;
    path.path.push_back(pose);
  }
  object.kinematics.initial_pose_with_covariance.pose = path.path.front();
  object.kinematics.predicted_paths = {path};
  const auto clear = find_predicted_crossings(ego, {object}, p);
  ASSERT_EQ(clear.cleared_objects.size(), 1U);
  EXPECT_TRUE(clear.collisions.empty());
  ObjectStopDecisionMap decisions;
  decisions[clear.cleared_objects.front()].update_timers(rclcpp::Time(0), 0, 0.3);
  EXPECT_TRUE(decisions.begin()->second.should_be_avoided());
  for (const auto & id : clear.cleared_objects) decisions.erase(id);
  EXPECT_TRUE(decisions.empty()); // No 0.3 s hold after a positive observation.
  EXPECT_TRUE(find_predicted_crossings(ego, {}, p).cleared_objects.empty());
  ego.prediction_age = 1.0;
  EXPECT_TRUE(find_predicted_crossings(ego, {object}, p).cleared_objects.empty());
}

TEST(PredictedCrossing, EntryCrossTrafficUsesCurrentBodyNotWholePrediction)
{
  geometry_msgs::msg::Pose ego;
  ego.position.x = 2496.0;
  ego.position.y = 24462.0;
  ego.orientation = autoware_utils::create_quaternion_from_yaw(M_PI_2);
  autoware_perception_msgs::msg::PredictedObject object;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;
  object.kinematics.initial_twist_with_covariance.twist.linear.x = 5.0;
  object.kinematics.initial_pose_with_covariance.pose.position.x = 2496.0;
  object.kinematics.initial_pose_with_covariance.pose.position.y = 24471.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation =
    autoware_utils::create_quaternion_from_yaw(M_PI_2);
  EXPECT_FALSE(autoware::motion_velocity_planner::roundabout_gap::is_entry_crossing_vehicle(object, ego));
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.z = 0.0;
  ASSERT_TRUE(autoware::motion_velocity_planner::roundabout_gap::is_entry_crossing_vehicle(object, ego));
  EXPECT_TRUE(autoware::motion_velocity_planner::roundabout_gap::occupies_entry_zone(object));
  object.kinematics.initial_pose_with_covariance.pose.position.x = 2506.0;
  EXPECT_FALSE(autoware::motion_velocity_planner::roundabout_gap::occupies_entry_zone(object));
  object.kinematics.initial_pose_with_covariance.pose.position.y = 24500.0;
  EXPECT_FALSE(autoware::motion_velocity_planner::roundabout_gap::is_entry_crossing_vehicle(object, ego));
}

TEST(PredictedCrossing, EntryGateCommitsOnlyAfterStopped)
{
  autoware_perception_msgs::msg::PredictedObject object;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.kinematics.initial_twist_with_covariance.twist.linear.x = 5.0;
  // The center has passed the door, but the 4 m vehicle rear still overlaps it.
  object.kinematics.initial_pose_with_covariance.pose.position.x = 2498.20;
  object.kinematics.initial_pose_with_covariance.pose.position.y = 24471.0;
  EXPECT_TRUE(autoware::motion_velocity_planner::roundabout_gap::occupies_entry_gate(object));
  // Release only after the rear clears x=2496.22.
  object.kinematics.initial_pose_with_covariance.pose.position.x = 2498.25;
  EXPECT_FALSE(autoware::motion_velocity_planner::roundabout_gap::occupies_entry_gate(object));
  EntryStopLatch latch;
  EXPECT_TRUE(latch.update(true, true, 0.0, 0.05, 1));
  EXPECT_TRUE(latch.update(true, false, 1.0, 0.05, 2));
  EXPECT_TRUE(latch.update(true, false, 0.05, 0.05, 3));
  EXPECT_TRUE(latch.update(true, true, 0.05, 0.05, 4));
  EXPECT_FALSE(latch.update(true, true, 0.05, 0.05, 5));
  EXPECT_TRUE(latch.entry_approved);
  EXPECT_FALSE(latch.update(true, true, 0.0, 0.05, 3));
}

TEST(PredictedCrossing, HoldSameObjectAcrossBriefDropout)
{
  ObjectStopDecision decision;
  decision.update_timers(rclcpp::Time(0), 0.1, 0.3);
  decision.update_timers(rclcpp::Time(100000000LL), 0.1, 0.3);
  ASSERT_TRUE(decision.should_be_avoided());
  decision.collision_detected = false;
  decision.update_timers(rclcpp::Time(200000000LL), 0.1, 0.3);
  EXPECT_TRUE(decision.should_be_avoided());
  decision.collision_detected = true;
  decision.update_timers(rclcpp::Time(250000000LL), 0.1, 0.3);
  EXPECT_TRUE(decision.should_be_avoided());
  decision.collision_detected = false;
  decision.update_timers(rclcpp::Time(600000000LL), 0.1, 0.3);
  EXPECT_FALSE(decision.should_be_avoided());
}

TEST(PredictedCrossing, PerpendicularClearingAndParallel)
{
  EgoData ego;
  ego.velocity = 4.0;
  ego.pose.orientation.w = 1.0;
  PlannerParam p;
  p.time_horizon = 5.0;
  p.ego_longitudinal_offset = 3.0;
  p.ego_lateral_offset = 1.0;
  p.extra_object_width = 0.5;
  for (int x = 0; x <= 40; ++x) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.orientation.w = 1.0;
    point.pose.position.x = x;
    ego.trajectory.push_back(point);
  }
  make_ego_footprint_rtree(ego, p);
  autoware_perception_msgs::msg::PredictedObject object;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;
  object.kinematics.initial_pose_with_covariance.pose.position.x = 12.0;
  object.kinematics.initial_pose_with_covariance.pose.position.y = -8.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation =
    autoware_utils::create_quaternion_from_yaw(M_PI_2);
  object.kinematics.initial_twist_with_covariance.twist.linear.x = 4.0;
  EXPECT_FALSE(find_predicted_crossings(ego, {object}, p).collisions.empty());
  ego.velocity = 0.0;
  EXPECT_FALSE(find_predicted_crossings(ego, {object}, p).collisions.empty());
  ego.velocity = 4.0;
  // The next car reaches this crossing after ego has passed it: do not add vehicle
  // lengths as a second occupancy duration on top of the footprint intersection.
  object.kinematics.initial_pose_with_covariance.pose.position.y = -18.0;
  EXPECT_TRUE(find_predicted_crossings(ego, {object}, p).collisions.empty());
  // Already moving away across the far side must not hold a stop indefinitely.
  object.kinematics.initial_pose_with_covariance.pose.position.y = 8.0;
  EXPECT_TRUE(find_predicted_crossings(ego, {object}, p).collisions.empty());
  object.shape.dimensions.x = 0.0;
  EXPECT_TRUE(find_predicted_crossings(ego, {object}, p).collisions.empty());
  object.shape.dimensions.x = 4.0;
  object.kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.z = 0.0;
  EXPECT_TRUE(find_predicted_crossings(ego, {object}, p).collisions.empty());
  // Empty inputs are valid.
  EXPECT_TRUE(find_predicted_crossings(EgoData{}, {object}, p).collisions.empty());
}

TEST(PredictedCrossing, CurvedPathAndPredictionAge)
{
  EgoData ego;
  ego.velocity = 3.0;
  PlannerParam p;
  p.time_horizon = 5.0;
  p.ego_longitudinal_offset = 3.0;
  p.ego_lateral_offset = 1.0;
  for (int x = 0; x < 30; ++x) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.orientation.w = 1.0;
    ego.trajectory.push_back(point);
  }
  make_ego_footprint_rtree(ego, p);
  autoware_perception_msgs::msg::PredictedObject object;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;
  object.kinematics.initial_pose_with_covariance.pose.position.x = 12.0;
  object.kinematics.initial_pose_with_covariance.pose.position.y = -10.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  autoware_perception_msgs::msg::PredictedPath path;
  path.confidence = 1.0;
  path.time_step.nanosec = 500000000;
  for (int i = 0; i <= 10; ++i) {
    auto pose = object.kinematics.initial_pose_with_covariance.pose;
    pose.position.y += i * 2.0;
    pose.orientation = autoware_utils::create_quaternion_from_yaw(M_PI_2);
    path.path.push_back(pose);
  }
  object.kinematics.predicted_paths.push_back(path);
  EXPECT_FALSE(find_predicted_crossings(ego, {object}, p).collisions.empty());
  // Cluster poses can carry ego yaw: actual movement is lateral in that frame.
  object.shape.type = autoware_perception_msgs::msg::Shape::POLYGON;
  object.shape.dimensions.x = object.shape.dimensions.y = 0.0;
  for (const auto & xy : {std::pair<float, float>{-2, -1}, {2, -1}, {2, 1}, {-2, 1}}) {
    geometry_msgs::msg::Point32 point;
    point.x = xy.first;
    point.y = xy.second;
    object.shape.footprint.points.push_back(point);
  }
  for (auto & pose : object.kinematics.predicted_paths.front().path) {
    pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
  }
  EXPECT_FALSE(find_predicted_crossings(ego, {object}, p).collisions.empty());
  object.kinematics.predicted_paths.clear();
  object.kinematics.initial_twist_with_covariance.twist.linear.y = 4.0;
  EXPECT_FALSE(find_predicted_crossings(ego, {object}, p).collisions.empty());
  object.shape.footprint.points.clear();
  EXPECT_TRUE(find_predicted_crossings(ego, {object}, p).collisions.empty());
  ego.prediction_age = 10.0;
  EXPECT_TRUE(find_predicted_crossings(ego, {object}, p).collisions.empty());
}
