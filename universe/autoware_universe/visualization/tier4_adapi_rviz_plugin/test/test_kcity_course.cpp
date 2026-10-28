// Copyright 2026 FML
// Licensed under the Apache License, Version 2.0.
#include "checkpoint_progress.hpp"
#include "kcity_course.hpp"

#include <gtest/gtest.h>
namespace course = tier4_adapi_rviz_plugins::kcity;

TEST(KCityCourse, UtmAndMgrs)
{
  const auto utm = course::pose(0, false);
  const auto map = course::pose(0);
  EXPECT_NEAR(utm.position.x, 302463.3102024494, 1e-7);
  EXPECT_NEAR(utm.position.y, 4123716.668977062, 1e-7);
  EXPECT_NEAR(map.position.x, 2463.3102024494, 1e-7);
  EXPECT_NEAR(map.position.y, 23716.668977062, 1e-7);
  EXPECT_DOUBLE_EQ(map.position.z, course::course[0].z);
  EXPECT_NEAR(std::hypot(map.orientation.z, map.orientation.w), 1.0, 1e-12);
  EXPECT_NEAR(
    2 * std::atan2(map.orientation.z, map.orientation.w) * 180 / std::acos(-1.0),
    course::course[0].yaw_deg, 1e-9);
}

TEST(KCityCourse, OrderedRoutesWithoutRepeatedStart)
{
  const auto initial = course::request(false, 1);
  ASSERT_EQ(initial.waypoints.size(), 13U);
  EXPECT_EQ(initial.header.frame_id, "map");
  EXPECT_FALSE(initial.option.allow_goal_modification);
  EXPECT_EQ(initial.goal, course::pose(14));
  for (std::size_t i = 0; i < 13; ++i) EXPECT_EQ(initial.waypoints[i], course::pose(i + 1));
  const auto extended = course::request(true, 3);
  ASSERT_EQ(extended.waypoints.size(), 13U);
  EXPECT_EQ(extended.goal, course::stop_goal_pose());
  for (std::size_t i = 0; i < 12; ++i) EXPECT_EQ(extended.waypoints[i], course::pose(i + 3));
  EXPECT_EQ(extended.waypoints.back(), course::pose(0));
  ASSERT_EQ(course::request(true, 15).waypoints.size(), 1U);
  EXPECT_EQ(course::request(true, 15).waypoints.front(), course::pose(0));
  EXPECT_NEAR(course::distance(extended.goal, course::pose(0)), 60.0, 0.1);
  EXPECT_NEAR(extended.goal.position.x, 2492.4680442334516, 1e-7);
  EXPECT_NEAR(extended.goal.position.y, 23769.107478781803, 1e-7);
  EXPECT_NEAR(course::pose(0).position.x, 2463.3102024494, 1e-7);
  EXPECT_THROW(course::request(true, 0), std::out_of_range);
}

TEST(KCityCourse, ProgressRejectsWrongCheckpointHeadingAndTeleports)
{
  auto target = course::pose(1);
  auto previous = target;
  previous.position.x -= 1;
  EXPECT_TRUE(course::passed(previous, target, 1));
  EXPECT_FALSE(course::passed(previous, target, 2));
  EXPECT_FALSE(course::passed(previous, target, 15));
  previous.position.x -= 100;
  EXPECT_FALSE(course::passed(previous, target, 1));
  previous = target;
  const auto q = target.orientation;
  target.orientation.z = q.w;
  target.orientation.w = -q.z;
  EXPECT_FALSE(course::passed(previous, target, 1));
}

TEST(KCityCourse, FastGateCrossing)
{
  auto before = course::pose(6), after = before;
  const double yaw = course::course[6].yaw_deg * std::acos(-1.0) / 180.0;
  before.position.x -= 10 * std::cos(yaw);
  before.position.y -= 10 * std::sin(yaw);
  after.position.x += 10 * std::cos(yaw);
  after.position.y += 10 * std::sin(yaw);
  EXPECT_TRUE(course::passed(before, after, 6));
  EXPECT_FALSE(course::passed(after, before, 6));
}

namespace
{
geometry_msgs::msg::Pose offset(std::size_t index, double along, double lateral)
{
  auto p = course::pose(index);
  const double yaw = course::course[index].yaw_deg * std::acos(-1.0) / 180.0;
  p.position.x += along * std::cos(yaw) - lateral * std::sin(yaw);
  p.position.y += along * std::sin(yaw) + lateral * std::cos(yaw);
  return p;
}
}  // namespace

TEST(CheckpointProgress, ThreeMetresAndSampleInterpolation)
{
  course::CheckpointProgress progress;
  const auto start = course::pose(0);
  EXPECT_FALSE(progress.update(start, start, 0.0));  // START is not END on departure.
  EXPECT_EQ(progress.successes(), 0U);
  EXPECT_TRUE(progress.update(offset(1, -5, 3.0), offset(1, 5, 3.0), 10.0));
  EXPECT_EQ(progress.results[1], course::CheckpointResult::Success);
  EXPECT_EQ(progress.results[2], course::CheckpointResult::Pending);
  progress.reset();
  EXPECT_TRUE(progress.update(offset(1, -5, 3.01), offset(1, 5, 3.01), 10.0));
  EXPECT_EQ(progress.results[1], course::CheckpointResult::Missed);
  EXPECT_EQ(progress.successes(), 0U);
}

TEST(CheckpointProgress, ApproachStaysGreyAndSkippedCheckpointIsRed)
{
  course::CheckpointProgress progress;
  EXPECT_FALSE(progress.update(offset(1, -10, 4), offset(1, -4, 4), 3.0));
  EXPECT_EQ(progress.results[1], course::CheckpointResult::Pending);
  auto second = course::pose(2);
  EXPECT_TRUE(progress.update(second, second, 3.0));
  EXPECT_EQ(progress.results[1], course::CheckpointResult::Missed);
  EXPECT_EQ(progress.results[2], course::CheckpointResult::Success);
  EXPECT_EQ(progress.results[3], course::CheckpointResult::Pending);
}

TEST(CheckpointProgress, EndThenStopAndRestartResetsWithoutUserAction)
{
  course::CheckpointProgress progress;
  for (std::size_t i = 1; i < 15; ++i) {
    const auto p = course::pose(i);
    EXPECT_TRUE(progress.update(p, p, 5.0));
  }
  EXPECT_EQ(progress.successes(), 14U);
  const auto end = course::pose(0);
  EXPECT_TRUE(progress.update(end, end, 1.0));
  EXPECT_EQ(progress.successes(), 15U);
  EXPECT_TRUE(progress.finished());
  EXPECT_FALSE(progress.update(end, end, 1.0));  // Do not erase on approach to stop.
  EXPECT_FALSE(progress.update(end, end, 0.0));
  EXPECT_FALSE(progress.update(end, end, 0.3));  // Reject standstill speed noise.
  EXPECT_TRUE(progress.update(end, end, 0.6));
  EXPECT_EQ(progress.successes(), 0U);
  EXPECT_EQ(progress.next, 1U);
  EXPECT_FALSE(progress.finished());
  for (const auto result : progress.results) EXPECT_EQ(result, course::CheckpointResult::Pending);
}

TEST(CheckpointProgress, TeleportAndInvalidCoordinatesDoNotScore)
{
  course::CheckpointProgress progress;
  EXPECT_FALSE(progress.update(offset(1, -100, 0), course::pose(1), 10.0));
  auto invalid = course::pose(1);
  invalid.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(progress.update(invalid, course::pose(1), 10.0));
  EXPECT_EQ(progress.successes(), 0U);
}
