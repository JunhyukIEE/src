#include "map_based_prediction/unknown_lane.hpp"
#include <gtest/gtest.h>

TEST(UnknownLane, ColdVelocityRequiresConsistentMeasuredMotion)
{
  using namespace autoware::map_based_prediction;
  autoware_perception_msgs::msg::TrackedObject object;
  object.kinematics.pose_with_covariance.pose.orientation.w = 1.0;
  UnknownMotionHistory history;
  for (int i = 0; i < 4; ++i) {
    object.kinematics.pose_with_covariance.pose.position.x = i * 0.5;
    recover_unknown_velocity(object, history, i * 0.1);
  }
  EXPECT_NEAR(object.kinematics.twist_with_covariance.twist.linear.x, 5.0, 1e-6);
  history.clear();
  object.kinematics.twist_with_covariance.twist.linear.x = 0.0;
  for (int i = 0; i < 4; ++i) {
    object.kinematics.pose_with_covariance.pose.position.x = i % 2;
    recover_unknown_velocity(object, history, i * 0.1);
  }
  EXPECT_DOUBLE_EQ(object.kinematics.twist_with_covariance.twist.linear.x, 0.0);
}

TEST(UnknownLane, RegionMotionShapeAndSemanticPreservation)
{
  using namespace autoware::map_based_prediction;
  autoware_perception_msgs::msg::TrackedObject object;
  object.classification.emplace_back();
  object.classification.front().probability = 1.0;
  auto & pose = object.kinematics.pose_with_covariance.pose;
  pose.position.x = 2490;
  pose.position.y = 24480;
  pose.orientation.w = 1.0;
  object.kinematics.twist_with_covariance.twist.linear.y = 5.0;
  object.shape.type = object.shape.POLYGON;
  for (const auto & xy : {std::pair<float, float>{-2,-1}, {2,-1}, {2,1}, {-2,1}}) {
    geometry_msgs::msg::Point32 p;
    p.x = xy.first; p.y = xy.second;
    object.shape.footprint.points.push_back(p);
  }
  const std::vector<double> region{2470,2525,24465,24510};
  auto candidate = unknown_lane_candidate(object, region);
  ASSERT_TRUE(candidate);
  EXPECT_EQ(candidate->classification.front().label, 0);
  EXPECT_NEAR(tf2::getYaw(candidate->kinematics.pose_with_covariance.pose.orientation), M_PI_2, 1e-6);
  EXPECT_DOUBLE_EQ(candidate->kinematics.twist_with_covariance.twist.linear.x, 5.0);
  const auto before = autoware_utils::to_polygon2d(object);
  const auto after = autoware_utils::to_polygon2d(*candidate);
  EXPECT_NEAR(boost::geometry::area(before), boost::geometry::area(after), 1e-6);
  for (size_t i=0;i<before.outer().size();++i) {
    EXPECT_NEAR(before.outer()[i].x(), after.outer()[i].x(), 1e-4);
    EXPECT_NEAR(before.outer()[i].y(), after.outer()[i].y(), 1e-4);
  }
  EXPECT_FALSE(unknown_lane_candidate(object, {}));
  pose.position.x = 2600;
  EXPECT_FALSE(unknown_lane_candidate(object, region));
  pose.position.x = 2490;
  object.kinematics.twist_with_covariance.twist.linear.y = 0.2;
  EXPECT_FALSE(unknown_lane_candidate(object, region));
}
