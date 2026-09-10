// Copyright 2023-2024 TIER IV, Inc. All rights reserved.
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

#include "dynamic_obstacle_stop_module.hpp"

#include "collision.hpp"
#include "crossing.hpp"
#include "debug.hpp"
#include "footprint.hpp"
#include "object_filtering.hpp"
#include "object_stop_decision.hpp"
#include "types.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_velocity_planner_common/roundabout_gap.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/system/stop_watch.hpp>

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{

void DynamicObstacleStopModule::init(rclcpp::Node & node, const std::string & module_name)
{
  module_name_ = module_name;
  logger_ = node.get_logger().get_child(ns_);
  clock_ = node.get_clock();

  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "dynamic_obstacle_stop");

  debug_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/debug_markers", 1);
  virtual_wall_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/virtual_walls", 1);
  processing_diag_publisher_ = std::make_shared<autoware_utils::ProcessingTimePublisher>(
    &node, "~/debug/" + ns_ + "/processing_time_ms_diag");
  processing_time_publisher_ =
    node.create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/" + ns_ + "/processing_time_ms", 1);

  using autoware_utils::get_or_declare_parameter;
  auto & p = params_;
  p.extra_object_width = get_or_declare_parameter<double>(node, ns_ + ".extra_object_width");
  p.minimum_object_velocity =
    get_or_declare_parameter<double>(node, ns_ + ".minimum_object_velocity");
  p.stop_distance_buffer = get_or_declare_parameter<double>(node, ns_ + ".stop_distance_buffer");
  p.time_horizon = get_or_declare_parameter<double>(node, ns_ + ".time_horizon");
  p.hysteresis = get_or_declare_parameter<double>(node, ns_ + ".hysteresis");
  p.add_duration_buffer = get_or_declare_parameter<double>(node, ns_ + ".add_stop_duration_buffer");
  p.remove_duration_buffer =
    get_or_declare_parameter<double>(node, ns_ + ".remove_stop_duration_buffer");
  p.minimum_object_distance_from_ego_trajectory =
    get_or_declare_parameter<double>(node, ns_ + ".minimum_object_distance_from_ego_trajectory");
  p.ignore_unavoidable_collisions =
    get_or_declare_parameter<bool>(node, ns_ + ".ignore_unavoidable_collisions");
  p.use_predicted_crossing_paths =
    node.declare_parameter<bool>(ns_ + ".use_predicted_crossing_paths", false);
  p.crossing_time_margin = node.declare_parameter<double>(ns_ + ".crossing_time_margin", 1.0);
  p.crossing_min_angle = node.declare_parameter<double>(ns_ + ".crossing_min_angle", 0.523599);
  p.approach_velocity = node.declare_parameter<double>(ns_ + ".approach_velocity", 3.3);
  p.approach_distance = node.declare_parameter<double>(ns_ + ".approach_distance", 30.0);
  p.departure_acceleration = node.declare_parameter<double>(ns_ + ".departure_acceleration", 2.0);

  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  p.ego_lateral_offset =
    std::max(std::abs(vehicle_info.min_lateral_offset_m), vehicle_info.max_lateral_offset_m);
  p.ego_longitudinal_offset = vehicle_info.max_longitudinal_offset_m;
}

void DynamicObstacleStopModule::update_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;
  auto & p = params_;
  update_param(parameters, ns_ + ".extra_object_width", p.extra_object_width);
  update_param(parameters, ns_ + ".minimum_object_velocity", p.minimum_object_velocity);
  update_param(parameters, ns_ + ".stop_distance_buffer", p.stop_distance_buffer);
  update_param(parameters, ns_ + ".time_horizon", p.time_horizon);
  update_param(parameters, ns_ + ".hysteresis", p.hysteresis);
  update_param(parameters, ns_ + ".add_stop_duration_buffer", p.add_duration_buffer);
  update_param(parameters, ns_ + ".remove_stop_duration_buffer", p.remove_duration_buffer);
  update_param(
    parameters, ns_ + ".minimum_object_distance_from_ego_trajectory",
    p.minimum_object_distance_from_ego_trajectory);
  update_param(parameters, ns_ + ".ignore_unavoidable_collisions", p.ignore_unavoidable_collisions);
  update_param(parameters, ns_ + ".use_predicted_crossing_paths", p.use_predicted_crossing_paths);
  update_param(parameters, ns_ + ".crossing_time_margin", p.crossing_time_margin);
  update_param(parameters, ns_ + ".crossing_min_angle", p.crossing_min_angle);
  update_param(parameters, ns_ + ".approach_velocity", p.approach_velocity);
  update_param(parameters, ns_ + ".approach_distance", p.approach_distance);
  update_param(parameters, ns_ + ".departure_acceleration", p.departure_acceleration);
}

void DynamicObstacleStopModule::publish_processing_time(const double processing_time_ms)
{
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = clock_->now();
  processing_time_msg.data = processing_time_ms;
  processing_time_publisher_->publish(processing_time_msg);
}

VelocityPlanningResult DynamicObstacleStopModule::plan(
  [[maybe_unused]] const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &
    raw_trajectory_points,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  VelocityPlanningResult result;
  debug_data_.reset_data();
  autoware_utils::StopWatch<std::chrono::microseconds> stopwatch;
  if (smoothed_trajectory_points.size() < 2) {
    publish_processing_time(stopwatch.toc() / 1000);
    return result;
  }

  if (roundabout_gap::in_entry_stop_arm_region(
        planner_data->current_odometry.pose.pose.position)) {
    // The behavior StopLine module exclusively owns the fixed entry stop and gate release.
    object_map_.clear();
    entry_stop_latch_ = {};
    entry_stop_limit_active_ = false;
    roundabout_entry_holding_ = false;
    publish_processing_time(stopwatch.toc() / 1000);
    return result;
  }

  stopwatch.tic();
  stopwatch.tic("preprocessing");
  dynamic_obstacle_stop::EgoData ego_data;
  ego_data.pose = planner_data->current_odometry.pose.pose;
  ego_data.velocity = planner_data->current_odometry.twist.twist.linear.x;
  ego_data.prediction_age =
    (clock_->now() - rclcpp::Time(planner_data->predicted_objects_header.stamp)).seconds();
  ego_data.trajectory = smoothed_trajectory_points;
  ego_data.trajectory = autoware::motion_utils::removeOverlapPoints(ego_data.trajectory);
  ego_data.first_trajectory_idx =
    autoware::motion_utils::findNearestSegmentIndex(ego_data.trajectory, ego_data.pose.position);
  ego_data.longitudinal_offset_to_first_trajectory_idx =
    autoware::motion_utils::calcLongitudinalOffsetToSegment(
      ego_data.trajectory, ego_data.first_trajectory_idx, ego_data.pose.position);
  const auto min_stop_distance = autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
                                   planner_data->current_odometry.twist.twist.linear.x, 0.0,
                                   planner_data->current_acceleration.accel.accel.linear.x,
                                   planner_data->velocity_smoother_->getMinDecel(),
                                   planner_data->velocity_smoother_->getMaxJerk(),
                                   planner_data->velocity_smoother_->getMinJerk())
                                   .value_or(0.0);
  ego_data.earliest_stop_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
    ego_data.trajectory, ego_data.pose.position, min_stop_distance);
  const bool in_entry = params_.use_predicted_crossing_paths &&
    roundabout_gap::in_entry_stop_arm_region(ego_data.pose.position);

  dynamic_obstacle_stop::make_ego_footprint_rtree(ego_data, params_);
  double hysteresis =
    std::find_if(
      object_map_.begin(), object_map_.end(),
      [](const auto & pair) { return pair.second.should_be_avoided(); }) == object_map_.end()
      ? 0.0
      : params_.hysteresis;
  const auto dynamic_obstacles = dynamic_obstacle_stop::filter_predicted_objects(
    planner_data->objects, ego_data, params_, hysteresis);

  const auto preprocessing_duration_us = stopwatch.toc("preprocessing");

  stopwatch.tic("footprints");
  const auto obstacle_forward_footprints =
    dynamic_obstacle_stop::make_forward_footprints(dynamic_obstacles, params_, hysteresis);
  const auto footprints_duration_us = stopwatch.toc("footprints");
  stopwatch.tic("collisions");
  auto collisions = dynamic_obstacle_stop::find_collisions(
    ego_data, dynamic_obstacles, obstacle_forward_footprints);
  if (params_.use_predicted_crossing_paths && !in_entry) {
    std::vector<autoware_perception_msgs::msg::PredictedObject> all_objects;
    for (const auto & object : planner_data->objects)
      all_objects.push_back(object->predicted_object);
    const auto crossing =
      dynamic_obstacle_stop::find_predicted_crossings(ego_data, all_objects, params_);
    collisions = crossing.collisions;
    // Fresh positive clearance is not a sensor dropout: release on this cycle.
    for (const auto & id : crossing.cleared_objects) object_map_.erase(id);
    if (crossing.approach_point) {
      result.slowdown_intervals.emplace_back(
        ego_data.pose.position, *crossing.approach_point, params_.approach_velocity);
      auto approach_pose = ego_data.pose;
      approach_pose.position = *crossing.approach_point;
      planning_factor_interface_->add(
        smoothed_trajectory_points, ego_data.pose, ego_data.pose, approach_pose,
        PlanningFactor::SLOW_DOWN, SafetyFactorArray{}, planner_data->is_driving_forward,
        params_.approach_velocity);
    }
  }
  bool gate_occupied = false;
  if (in_entry) {
    // The fixed gate is the only object condition in the competition entry region.
    gate_occupied = std::any_of(
      planner_data->objects.begin(), planner_data->objects.end(), [](const auto & object) {
        return roundabout_gap::occupies_entry_gate(object->predicted_object);
      });
    collisions.clear();
    object_map_.clear();
  } else {
    update_object_map(object_map_, collisions, clock_->now(), ego_data.trajectory, params_);
  }
  std::optional<geometry_msgs::msg::Point> earliest_collision;
  if (!in_entry) {
    earliest_collision = dynamic_obstacle_stop::find_earliest_collision(object_map_, ego_data);
  }
  const bool was_holding = entry_stop_latch_.holding;
  const bool had_seen_gate_vehicle = entry_stop_latch_.saw_gate_vehicle;
  // A brief stop upstream must not satisfy the mandatory stop requirement.
  const double latch_speed = roundabout_gap::has_reached_entry_waiting_point(ego_data.pose.position)
    ? ego_data.velocity
    : 1.0;
  const bool hold_entry = entry_stop_latch_.update(
    in_entry, gate_occupied, latch_speed, ego_data.prediction_age,
    rclcpp::Time(planner_data->predicted_objects_header.stamp).nanoseconds());
  roundabout_entry_holding_ = in_entry && hold_entry;
  // The fixed gate is the sole longitudinal authority through the commit region.
  result.suppress_other_obstacle_results = in_entry;
  result.apply_roundabout_entry_launch = in_entry && entry_stop_latch_.entry_approved;
  if (in_entry && entry_stop_latch_.entry_approved) {
    // The gap was accepted. Do not revive a crossing stop until the ego clears this entry.
    object_map_.clear();
  }
  if (in_entry && was_holding != hold_entry) {
    RCLCPP_INFO(logger_, "[roundabout_entry] %s speed=%.2f prediction_age=%.3f",
      hold_entry ? "BRAKE/HOLD" : "ENTERING (approval retained)",
      ego_data.velocity, ego_data.prediction_age);
  }
  if (in_entry && !had_seen_gate_vehicle && entry_stop_latch_.saw_gate_vehicle) {
    RCLCPP_INFO(logger_, "[roundabout_entry] GATE_SEEN: waiting for rear clear");
  }
  const auto collisions_duration_us = stopwatch.toc("collisions");
  if (earliest_collision || hold_entry) {
    // Keep the requested stop before the conflict even when detection was late.
    // Moving the stop to the earliest comfortable stop can put it inside the crossing.
    double stop_distance = std::numeric_limits<double>::infinity();
    if (earliest_collision) {
      stop_distance = autoware::motion_utils::calcSignedArcLength(
        ego_data.trajectory, ego_data.pose.position, *earliest_collision) -
        params_.stop_distance_buffer -
        (params_.use_predicted_crossing_paths ? 0.0 : params_.ego_longitudinal_offset);
    }
    if (hold_entry) {
      // Stop on the fixed cross-lane line, never on an arbitrary map point near it.
      const auto waiting_point = roundabout_gap::entry_stop_line_intersection(
        ego_data.trajectory, ego_data.pose.position);
      if (waiting_point) {
        const double waiting_distance = autoware::motion_utils::calcSignedArcLength(
          ego_data.trajectory, ego_data.pose.position, *waiting_point);
        stop_distance = std::min(stop_distance, waiting_distance);
        if (roundabout_gap::has_passed_entry_waiting_point(waiting_distance)) {
          // A trajectory point cannot stay behind ego. Hold zero speed until the gate clears instead.
          autoware_internal_planning_msgs::msg::VelocityLimit limit;
          limit.stamp = clock_->now();
          limit.sender = "roundabout_entry_stop";
          limit.max_velocity = 0.0F;
          result.velocity_limit = limit;
          entry_stop_limit_active_ = true;
        }
      } else {
        // The route has no crossing with the configured stop line: do not enter blindly.
        stop_distance = 0.0;
        autoware_internal_planning_msgs::msg::VelocityLimit limit;
        limit.stamp = clock_->now();
        limit.sender = "roundabout_entry_stop";
        limit.max_velocity = 0.0F;
        result.velocity_limit = limit;
        entry_stop_limit_active_ = true;
      }
    }
    const auto stop_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
      ego_data.trajectory, ego_data.pose.position,
      std::max(0.0, stop_distance));
    debug_data_.stop_pose = stop_pose;
    if (stop_pose) {
      result.stop_points.push_back(stop_pose->position);
      planning_factor_interface_->add(
        smoothed_trajectory_points, ego_data.pose, *stop_pose, PlanningFactor::STOP,
        SafetyFactorArray{});
      create_virtual_walls();
    }
  }
  if (!hold_entry && entry_stop_limit_active_) {
    autoware_internal_planning_msgs::msg::VelocityLimitClearCommand clear;
    clear.stamp = clock_->now();
    clear.sender = "roundabout_entry_stop";
    clear.command = true;
    result.velocity_limit_clear_command = clear;
    entry_stop_limit_active_ = false;
  }

  debug_publisher_->publish(create_debug_marker_array());
  virtual_wall_publisher_->publish(virtual_wall_marker_creator.create_markers());

  const auto total_time_us = stopwatch.toc();
  RCLCPP_DEBUG(
    logger_,
    "Total time = %2.2fus\n\tpreprocessing = %2.2fus\n\tfootprints = "
    "%2.2fus\n\tcollisions = %2.2fus\n",
    total_time_us, preprocessing_duration_us, footprints_duration_us, collisions_duration_us);
  debug_data_.ego_footprints = ego_data.trajectory_footprints;
  debug_data_.obstacle_footprints = obstacle_forward_footprints;
  debug_data_.z = ego_data.pose.position.z;
  std::map<std::string, double> processing_times;
  processing_times["preprocessing"] = preprocessing_duration_us / 1000;
  processing_times["footprints"] = footprints_duration_us / 1000;
  processing_times["collisions"] = collisions_duration_us / 1000;
  processing_times["Total"] = total_time_us / 1000;
  processing_diag_publisher_->publish(processing_times);
  publish_processing_time(processing_times["Total"]);
  return result;
}

visualization_msgs::msg::MarkerArray DynamicObstacleStopModule::create_debug_marker_array()
{
  const auto z = debug_data_.z;
  visualization_msgs::msg::MarkerArray array;
  std::string ns = "collisions";
  const auto collision_markers =
    dynamic_obstacle_stop::debug::make_collision_markers(object_map_, ns, z, clock_->now());
  dynamic_obstacle_stop::debug::add_markers(
    array, debug_data_.prev_collisions_nb, collision_markers, ns);
  ns = "dynamic_obstacles_footprints";
  const auto obstacle_footprint_markers =
    dynamic_obstacle_stop::debug::make_polygon_markers(debug_data_.obstacle_footprints, ns, z);
  dynamic_obstacle_stop::debug::add_markers(
    array, debug_data_.prev_dynamic_obstacles_nb, obstacle_footprint_markers, ns);
  ns = "ego_footprints";
  const auto ego_footprint_markers =
    dynamic_obstacle_stop::debug::make_polygon_markers(debug_data_.ego_footprints, ns, z);
  dynamic_obstacle_stop::debug::add_markers(
    array, debug_data_.prev_ego_footprints_nb, ego_footprint_markers, ns);
  return array;
}

void DynamicObstacleStopModule::create_virtual_walls()
{
  if (debug_data_.stop_pose) {
    autoware::motion_utils::VirtualWall virtual_wall;
    virtual_wall.text = roundabout_entry_holding_ ? "roundabout_entry_stop" : "dynamic_obstacle_stop";
    virtual_wall.longitudinal_offset = params_.ego_longitudinal_offset;
    virtual_wall.style = autoware::motion_utils::VirtualWallType::stop;
    virtual_wall.pose = *debug_data_.stop_pose;
    virtual_wall_marker_creator.add_virtual_wall(virtual_wall);
  }
}

}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::DynamicObstacleStopModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
