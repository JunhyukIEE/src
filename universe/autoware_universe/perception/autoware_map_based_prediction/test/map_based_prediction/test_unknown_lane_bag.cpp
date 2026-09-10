#include "map_based_prediction/unknown_lane.hpp"
#include "map_based_prediction/utils.hpp"
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <lanelet2_io/Io.h>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <gtest/gtest.h>
#include <cstdlib>
#include <iostream>

TEST(UnknownLaneBag, ActualMapAndRecordedObjects)
{
  using namespace autoware::map_based_prediction;
  const auto bag = std::getenv("MORAI_UNKNOWN_LANE_BAG");
  const auto map_path = std::getenv("MORAI_LANELET_MAP");
  if (!bag || !map_path) GTEST_SKIP() << "Set MORAI_UNKNOWN_LANE_BAG and MORAI_LANELET_MAP";
  lanelet::projection::MGRSProjector projector;
  lanelet::ErrorMessages errors;
  const lanelet::LaneletMapPtr map = lanelet::load(map_path, "autoware_osm_handler", projector, &errors);
  ASSERT_FALSE(map->laneletLayer.empty());
  rosbag2_cpp::Reader reader;
  reader.open(bag);
  size_t unknown = 0, eligible = 0, lane_matched = 0;
  size_t recovered = 0;
  std::unordered_map<std::string, UnknownMotionHistory> motion;
  std::unordered_map<std::string, std::pair<double,double>> observations;
  std::unordered_map<std::string, std::deque<ObjectData>> history;
  while (reader.has_next()) {
    const auto message = reader.read_next();
    if (message->topic_name != "/perception/object_recognition/tracking/objects") continue;
    rclcpp::SerializedMessage serialized(*message->serialized_data);
    autoware_perception_msgs::msg::TrackedObjects objects;
    rclcpp::Serialization<autoware_perception_msgs::msg::TrackedObjects>().deserialize_message(
      &serialized, &objects);
    if (objects.header.frame_id != "map") continue;
    const double time = rclcpp::Time(objects.header.stamp).seconds();
    for (auto object : objects.objects) {
      if (object.classification.empty() || object.classification.front().label != 0) continue;
      ++unknown;
      const auto & pos = object.kinematics.pose_with_covariance.pose.position;
      if (pos.x < 2470 || pos.x > 2525 || pos.y < 24465 || pos.y > 24510) continue;
      const auto id = autoware_utils::to_hex_string(object.object_id);
      const double before = std::hypot(object.kinematics.twist_with_covariance.twist.linear.x,
        object.kinematics.twist_with_covariance.twist.linear.y);
      recover_unknown_velocity(object, motion[id], time);
      recovered += std::hypot(object.kinematics.twist_with_covariance.twist.linear.x,
        object.kinematics.twist_with_covariance.twist.linear.y) > before + 0.1;
      auto candidate = unknown_lane_candidate(object, {2470,2525,24465,24510});
      if (!candidate) continue;
      auto & times = observations[id];
      if (time - times.second > 0.5 || time < times.second) times.first = time;
      times.second = time;
      if (time - times.first < 0.3) continue;
      ++eligible;
      if (!utils::getCurrentLanelets(*candidate, map, history, 1.5, 0.5, 0.5, 5.0).empty())
        ++lane_matched;
    }
  }
  EXPECT_GT(lane_matched, 0U);
  std::cout << "unknown=" << unknown << " eligible=" << eligible
            << " lane_matched=" << lane_matched << " recovered=" << recovered << std::endl;
}
