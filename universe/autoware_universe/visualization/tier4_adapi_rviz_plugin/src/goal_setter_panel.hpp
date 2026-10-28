// Copyright 2026 FML
// Licensed under the Apache License, Version 2.0.
#ifndef GOAL_SETTER_PANEL_HPP_
#define GOAL_SETTER_PANEL_HPP_

#include "checkpoint_progress.hpp"
#include "kcity_speed_limit.hpp"

#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>

namespace tier4_adapi_rviz_plugins
{
class GoalSetterPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit GoalSetterPanel(QWidget * parent = nullptr);
  ~GoalSetterPanel() override;
  void onInitialize() override;

private:
  using Service = autoware_adapi_v1_msgs::srv::SetRoutePoints;
  using State = autoware_adapi_v1_msgs::msg::RouteState;
  using Mode = autoware_adapi_v1_msgs::msg::OperationModeState;
  using Clock = std::chrono::steady_clock;
  QPushButton * button_;
  QLabel * count_;
  QLabel * speed_label_;
  kcity::SpeedLimitWatch speed_watch_;
  QPushButton * reset_;
  std::array<QLabel *, kcity::course.size()> lights_{};
  kcity::CheckpointProgress progress_;
  bool tracking_progress_{false};
  QTimer * timer_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<Service>::SharedPtr set_, change_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_sub_;
  rclcpp::Subscription<State>::SharedPtr state_sub_;
  rclcpp::Subscription<Mode>::SharedPtr mode_sub_;
  rclcpp::Subscription<autoware_map_msgs::msg::MapProjectorInfo>::SharedPtr map_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr velocity_;
  Mode::ConstSharedPtr mode_;
  uint16_t route_state_{State::UNKNOWN};
  bool map_ok_{false}, active_{false}, pending_{false}, final_leg_{false};
  bool timed_out_{false};
  std::size_t next_{1};
  Clock::time_point odom_received_{}, last_attempt_{}, sent_at_{};
  Clock::time_point velocity_received_{};
  void tick();
  void start();
  void send(bool final_leg);
  void showMarkers();
  void showProgress();
  void showSpeedLimit();
  void stop(const QString & reason);
};
}  // namespace tier4_adapi_rviz_plugins
#endif  // GOAL_SETTER_PANEL_HPP_
