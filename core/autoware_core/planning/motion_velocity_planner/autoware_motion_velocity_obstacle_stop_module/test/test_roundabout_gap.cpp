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

#include <autoware/motion_velocity_planner_common/roundabout_gap.hpp>
#include <gtest/gtest.h>

TEST(RoundaboutGap, CurrentCarNextCarAndUnavailableEvidence)
{
  using namespace autoware::motion_velocity_planner;
  geometry_msgs::msg::Pose ego;
  ego.orientation = autoware_utils_geometry::create_quaternion_from_yaw(1.57079632679);
  ego.position.x = 2496; ego.position.y = 24460;
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> trajectory;
  std::vector<autoware_utils_geometry::Polygon2d> polygons;
  for(int i=0;i<35;++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose=ego; point.pose.position.y += i;
    trajectory.push_back(point);
    polygons.push_back(autoware_utils_geometry::to_footprint(point.pose,3.8,0.0,2.0));
  }
  autoware_perception_msgs::msg::PredictedObject object;
  object.shape.dimensions.x=4; object.shape.dimensions.y=2;
  object.kinematics.initial_pose_with_covariance.pose=ego;
  object.kinematics.initial_twist_with_covariance.twist.linear.x=5;
  const auto set_path=[&](double start_x) {
    autoware_perception_msgs::msg::PredictedPath path;
    path.confidence=1; path.time_step.nanosec=500000000;
    for(int i=0;i<21;++i) {
      auto pose=ego; pose.position.x=start_x+2.5*i; pose.position.y=24471;
      pose.orientation=autoware_utils_geometry::create_quaternion_from_yaw(0);
      path.path.push_back(pose);
    }
    object.kinematics.initial_pose_with_covariance.pose=path.path.front();
    object.kinematics.predicted_paths={path};
  };
  set_path(2496);
  auto current=roundabout_gap::clear(object,ego,0,0,trajectory,polygons);
  ASSERT_TRUE(current); EXPECT_TRUE(*current); // Clears before ego reaches the crossing.
  set_path(2481);
  geometry_msgs::msg::Point conflict;
  auto crossing=roundabout_gap::clear(object,ego,0,0,trajectory,polygons,&conflict);
  ASSERT_TRUE(crossing); EXPECT_FALSE(*crossing);
  EXPECT_GE(conflict.y, ego.position.y);
  set_path(2450);
  auto next=roundabout_gap::clear(object,ego,0,0,trajectory,polygons);
  ASSERT_TRUE(next); EXPECT_TRUE(*next);
  set_path(2508);
  auto departed=roundabout_gap::clear(object,ego,0,0,trajectory,polygons);
  ASSERT_TRUE(departed); EXPECT_TRUE(*departed);
  set_path(2491.5); // The swept rear footprint still reaches the ego path before ego clears it.
  auto overlap=roundabout_gap::clear(object,ego,0,0,trajectory,polygons);
  ASSERT_TRUE(overlap); EXPECT_FALSE(*overlap);
  set_path(2450);
  object.kinematics.predicted_paths.front().path.resize(2);
  EXPECT_FALSE(roundabout_gap::clear(object,ego,0,0,trajectory,polygons));
  EXPECT_FALSE(roundabout_gap::clear(object,ego,0,1,trajectory,polygons));
  object.kinematics.initial_twist_with_covariance.twist.linear.x=0;
  EXPECT_FALSE(roundabout_gap::clear(object,ego,0,0,trajectory,polygons));
  ego.position.x=2600;
  EXPECT_FALSE(roundabout_gap::clear(object,ego,0,0,trajectory,polygons));
}
