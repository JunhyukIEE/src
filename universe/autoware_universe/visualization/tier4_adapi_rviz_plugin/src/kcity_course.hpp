// Copyright 2026 FML
// Licensed under the Apache License, Version 2.0.
#ifndef KCITY_COURSE_HPP_
#define KCITY_COURSE_HPP_

#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace tier4_adapi_rviz_plugins::kcity
{
struct Checkpoint
{
  double x, y, z, yaw_deg;
};
// MORAI K-City local coordinates; index 0 is START/END, not the first waypoint.
inline constexpr std::array<Checkpoint, 15> course{
  {{-131.68979755061446, -428.3310229377821, 28.543960281954277, 61.298126221},
   {-96.34194502758328, -364.8851077128202, 28.53881740888334, 64.052734375},
   {-104.71588468790287, -275.14425209118053, 28.48829083410744, 60.959838867},
   {-79.9685367429629, -230.3942881319672, 28.401036010337744, 61.097732543},
   {-59.88466601172695, -124.55752767482772, 28.380910298729233, 90.095718383},
   {-60.34860723745078, -0.21668933285400271, 28.383039161118077, 90.321304321},
   {-41.70069433480967, 98.73265545303002, 28.358756220677893, 0.456359863},
   {2.9027890474535525, 114.94218392064795, 28.430466193056173, 91.440628051},
   {-99.44168814341538, 291.20583281712607, 28.379763120717023, 90.260665893},
   {-65.32824593910482, 342.2530235303566, 28.484, 3.828613281},
   {62.073913682601415, 256.4949275725521, 28.268789437406806, -89.012939453},
   {73.55110931396484, -238.7718963623047, 28.71942138671875, -95.719757080},
   {70.21414883446414, -365.74847655789927, 28.301585314776467, -90.049835205},
   {43.761510782991536, -476.41094469698146, 28.294397987369784, -137.400207519},
   {-74.86074459552765, -544.5780326710083, 28.429146234972457, -153.528732299}}};

inline geometry_msgs::msg::Pose pose_from_local(const Checkpoint & p, bool mgrs = true)
{
  geometry_msgs::msg::Pose result;
  // Local -> UTM zone 52N, then strip the 100-km grid origin for map 52SCG.
  result.position.x = p.x + 302595.0 - (mgrs ? 300000.0 : 0.0);
  result.position.y = p.y + 4124145.0 - (mgrs ? 4100000.0 : 0.0);
  result.position.z = p.z;
  const double half_yaw = p.yaw_deg * std::acos(-1.0) / 360.0;
  result.orientation.z = std::sin(half_yaw);
  result.orientation.w = std::cos(half_yaw);
  return result;
}

inline geometry_msgs::msg::Pose pose(std::size_t index, bool mgrs = true)
{
  return pose_from_local(course.at(index), mgrs);
}

inline geometry_msgs::msg::Pose stop_goal_pose()
{
  // 60 m beyond END along K_city_26_crosswork_1.osm's drivable centerline.
  // Deliberately separate from course[0], which remains the scored END checkpoint.
  return pose_from_local(
    {-102.53195576654844, -375.89252121819663, 28.533099783169355, 60.68718823065855});
}

inline double distance(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}

inline bool passed(
  const geometry_msgs::msg::Pose & previous, const geometry_msgs::msg::Pose & current,
  std::size_t next)
{
  if (next == 0 || next >= course.size() || distance(previous, current) > 30.0) return false;
  const auto target = pose(next);
  const double yaw = course[next].yaw_deg * std::acos(-1.0) / 180.0;
  const auto & q = current.orientation;
  const double current_yaw =
    std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  if (std::cos(current_yaw - yaw) < 0.25) return false;
  if (distance(current, target) <= 8.0) return true;
  const double dx = current.position.x - target.position.x;
  const double dy = current.position.y - target.position.y;
  const double along = dx * std::cos(yaw) + dy * std::sin(yaw);
  const double before = (previous.position.x - target.position.x) * std::cos(yaw) +
                        (previous.position.y - target.position.y) * std::sin(yaw);
  const double lateral = -dx * std::sin(yaw) + dy * std::cos(yaw);
  return before < 0.0 && along >= 0.0 && std::abs(lateral) <= 8.0;
}

inline autoware_adapi_v1_msgs::srv::SetRoutePoints::Request request(
  bool final_leg, std::size_t next)
{
  if (next == 0 || next > course.size()) throw std::out_of_range("Invalid next checkpoint");
  autoware_adapi_v1_msgs::srv::SetRoutePoints::Request result;
  result.header.frame_id = "map";
  result.option.allow_goal_modification = false;
  result.goal = final_leg ? stop_goal_pose() : pose(14);
  const auto end = final_leg ? course.size() : course.size() - 1;
  for (auto i = next; i < end; ++i) result.waypoints.push_back(pose(i));
  if (final_leg) result.waypoints.push_back(pose(0));
  return result;
}
}  // namespace tier4_adapi_rviz_plugins::kcity
#endif  // KCITY_COURSE_HPP_
