#include "autoware/multi_object_tracker/tracker/model/pass_through_tracker.hpp"
#include <gtest/gtest.h>
#include "autoware/multi_object_tracker/tracker/model/unknown_tracker.hpp"

using namespace autoware::multi_object_tracker;

TEST(UnknownMotion, PublishedPoseUsesRequestedTime)
{
  types::DynamicObject object;
  object.pose.orientation.w = 1.0;
  object.kinematics.has_twist = true;
  object.twist.linear.x = 5.0;
  object.shape.type = object.shape.BOUNDING_BOX;
  object.shape.dimensions.x = 4;
  object.shape.dimensions.y = 2;
  const rclcpp::Time start(1000000000LL);
  UnknownTracker tracker(start, object, true, true);
  types::DynamicObject published;
  ASSERT_TRUE(tracker.getTrackedObject(rclcpp::Time(1200000000LL), published, true));
  EXPECT_NEAR(published.pose.position.x, 1.0, 1e-3);
  EXPECT_NEAR(published.twist.linear.x, 5.0, 1e-3);
}
class ClassificationProbe : public PassThroughTracker
{
public:
  using PassThroughTracker::PassThroughTracker;
  using Tracker::updateClassification;
};

TEST(Classification, UnknownIsNotEvidenceAgainstTrustedClass)
{
  types::DynamicObject object;
  autoware_perception_msgs::msg::ObjectClassification label;
  label.label = label.UNKNOWN;
  label.probability = 1.0;
  object.classification = {label};
  ClassificationProbe tracker(rclcpp::Time(0), object);
  EXPECT_EQ(tracker.getHighestProbLabel(), label.UNKNOWN);
  label.label = label.CAR;
  tracker.updateClassification({label});
  EXPECT_EQ(tracker.getHighestProbLabel(), label.CAR);
  label.label = label.TRUCK;
  tracker.updateClassification({label});
  EXPECT_EQ(tracker.getHighestProbLabel(), label.CAR);  // keep known-class smoothing
}
