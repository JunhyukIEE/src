// Copyright 2026 FML
// Licensed under the Apache License, Version 2.0.
#include "goal_setter_panel.hpp"

#include "kcity_course.hpp"

#include <QDockWidget>
#include <QGridLayout>
#include <QMainWindow>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rviz_common/window_manager_interface.hpp>

#include <memory>
#include <string>

namespace tier4_adapi_rviz_plugins
{
GoalSetterPanel::GoalSetterPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  button_ = new QPushButton("Goal Setter: K-City 1 lap", this);
  button_->setEnabled(false);
  button_->setToolTip("Waiting for map, localization and route state");
  layout->addWidget(button_);
  speed_label_ = new QLabel(this);
  speed_label_->setObjectName("speed_limit_status");
  speed_label_->setAlignment(Qt::AlignCenter);
  speed_label_->setMinimumHeight(52);
  speed_label_->setToolTip(
    "MGRS 52SCG: 시작 (2657.074, 24401.495), 종료 (2665.214, 23779.252).\n"
    "두 지점 사이 직선 기준 좌우 20m를 무제한 구간으로 표시.\n"
    "그 외 60km/h 초과 시 경고. 차량 기준점(base_link)과 실제 차속 사용. 제어 개입 없음.");
  layout->addWidget(speed_label_);
  showSpeedLimit();
  count_ = new QLabel(this);
  layout->addWidget(count_);
  auto * grid = new QGridLayout;
  for (std::size_t slot = 0; slot < lights_.size(); ++slot) {
    const auto index = (slot + 1) % lights_.size();  // END follows checkpoint 14.
    lights_[index] = new QLabel(this);
    lights_[index]->setAlignment(Qt::AlignCenter);
    lights_[index]->setFixedSize(38, 38);
    grid->addWidget(
      lights_[index], static_cast<int>(slot / 5), static_cast<int>(slot % 5), Qt::AlignCenter);
  }
  layout->addLayout(grid);
  reset_ = new QPushButton("통과 기록 초기화", this);
  reset_->setEnabled(false);
  reset_->setToolTip("수동 모드·정차 시 사용. 경로와 Auto 상태는 변경하지 않습니다.");
  layout->addWidget(reset_);
  connect(reset_, &QPushButton::clicked, this, [this] {
    if (!reset_->isEnabled()) return;
    progress_.reset();
    tracking_progress_ = true;
    showProgress();
  });
  showProgress();
  timer_ = new QTimer(this);
  connect(button_, &QPushButton::clicked, this, &GoalSetterPanel::start);
  connect(timer_, &QTimer::timeout, this, &GoalSetterPanel::tick);
}

GoalSetterPanel::~GoalSetterPanel()
{
  timer_->stop();
  if (node_) executor_.remove_node(node_);
}

void GoalSetterPanel::onInitialize()
{
  const auto rviz_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  const bool sim_time = rviz_node->get_parameter("use_sim_time").as_bool();
  node_ = std::make_shared<rclcpp::Node>(
    "kcity_goal_setter", rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides(
                           {rclcpp::Parameter("use_sim_time", sim_time)}));
  executor_.add_node(node_);
  set_ = node_->create_client<Service>("/api/routing/set_route_points");
  change_ = node_->create_client<Service>("/api/routing/change_route_points");
  const auto latched = rclcpp::QoS(1).transient_local();
  markers_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/rviz/goal_setter/checkpoints", latched);
  map_sub_ = node_->create_subscription<autoware_map_msgs::msg::MapProjectorInfo>(
    "/map/map_projector_info", latched,
    [this](autoware_map_msgs::msg::MapProjectorInfo::ConstSharedPtr msg) {
      map_ok_ = msg->projector_type == "MGRS" && msg->mgrs_grid == "52SCG";
      if (!map_ok_) {
        stop("Map must use MGRS 52SCG. No route sent.");
      } else {
        showMarkers();
      }
    });
  mode_sub_ = node_->create_subscription<Mode>(
    "/api/operation_mode/state", latched, [this](Mode::ConstSharedPtr msg) { mode_ = msg; });
  state_sub_ = node_->create_subscription<State>(
    "/api/routing/state", latched, [this](State::ConstSharedPtr msg) {
      route_state_ = msg->state;
      if (active_ && !pending_ && route_state_ == State::UNSET)
        stop("Route cleared; mission cancelled");
      if (active_ && !pending_ && route_state_ == State::ARRIVED) {
        stop(
          final_leg_ && next_ == 15
            ? "Course complete: stop goal reached beyond END"
            : "Stopped at temporary goal or missed checkpoint. Mission incomplete.");
      }
    });
  route_sub_ = node_->create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "/planning/mission_planning/route", latched,
    [this](autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg) {
      if (
        active_ && !pending_ &&
        kcity::distance(msg->goal_pose, final_leg_ ? kcity::stop_goal_pose() : kcity::pose(14)) >
          10.0) {
        stop("External route change detected; automatic updates cancelled");
      }
    });
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", rclcpp::QoS(1).best_effort(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      const bool fresh = Clock::now() - odom_received_ < std::chrono::milliseconds(500);
      if (
        tracking_progress_ && map_ok_ && fresh && odom_ && odom_->header.frame_id == "map" &&
        msg->header.frame_id == "map" &&
        progress_.update(odom_->pose.pose, msg->pose.pose, msg->twist.twist.linear.x)) {
        showProgress();
      }
      if (
        active_ && fresh && odom_ && msg->header.frame_id == "map" &&
        kcity::passed(odom_->pose.pose, msg->pose.pose, next_)) {
        ++next_;
        showMarkers();
        if (!pending_)
          button_->setToolTip(
            QString("Route progress %1 / 14; %2")
              .arg(next_ - 1)
              .arg(final_leg_ ? "stop goal: 60 m beyond END" : "extending route automatically"));
      }
      odom_ = msg;
      odom_received_ = Clock::now();
      showSpeedLimit();
    });
  velocity_sub_ = node_->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS(1).best_effort(),
    [this](autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg) {
      velocity_ = msg;
      velocity_received_ = Clock::now();
      showSpeedLimit();
    });
  timer_->start(100);
  // Dock this new panel on the left without replacing the user's saved RViz layout.
  QTimer::singleShot(0, this, [this] {
    auto * dock = qobject_cast<QDockWidget *>(parentWidget());
    auto * window =
      qobject_cast<QMainWindow *>(getDisplayContext()->getWindowManager()->getParentWindow());
    if (dock && window) {
      window->addDockWidget(Qt::LeftDockWidgetArea, dock);
      dock->setFloating(false);
    }
  });
}

void GoalSetterPanel::tick()
{
  if (!rclcpp::ok()) return;
  executor_.spin_some();
  showSpeedLimit();
  const auto now = Clock::now();
  const bool fresh = odom_ && now - odom_received_ < std::chrono::milliseconds(500);
  const bool manual =
    mode_ && !(mode_->mode == Mode::AUTONOMOUS && mode_->is_autoware_control_enabled);
  reset_->setEnabled(!pending_ && fresh && manual && std::abs(odom_->twist.twist.linear.x) < 0.1);
  button_->setEnabled(
    !active_ && !pending_ && map_ok_ && fresh && manual && route_state_ == State::UNSET &&
    std::abs(odom_->twist.twist.linear.x) < 0.1 && set_->service_is_ready());
  if (pending_ && !timed_out_ && now - sent_at_ > std::chrono::seconds(10)) {
    timed_out_ = true;
    button_->setToolTip(
      "Route response delayed; keeping current route, waiting (no duplicate request)");
  }
  if (
    active_ && !pending_ && !final_leg_ && next_ >= 3 && map_ok_ && fresh &&
    odom_->header.frame_id == "map" && route_state_ == State::SET &&
    now - last_attempt_ >= std::chrono::seconds(2) && change_->service_is_ready()) {
    send(true);
  }
}

void GoalSetterPanel::showSpeedLimit()
{
  const auto now = Clock::now();
  const bool fresh = map_ok_ && odom_ && velocity_ && odom_->header.frame_id == "map" &&
                     now - odom_received_ < std::chrono::milliseconds(500) &&
                     now - velocity_received_ < std::chrono::milliseconds(500);
  const auto status = speed_watch_.update(
    odom_ ? odom_->pose.pose.position.x : 0.0, odom_ ? odom_->pose.pose.position.y : 0.0,
    velocity_ ? velocity_->longitudinal_velocity : 0.0,
    std::chrono::duration<double>(now.time_since_epoch()).count(), fresh);
  QString text = "속도 감시 · 데이터 대기";
  QString background = "#343c42", foreground = "#b0bac2";
  if (status.valid) {
    foreground = "#f0f4f6";
    text = QString("%1  |  %2 km/h")
             .arg(status.unlimited ? "속도 무제한" : "제한 60 km/h")
             .arg(status.kmh, 0, 'f', 2);
    background = status.unlimited ? "#245866" : "#344b42";
    if (status.warning) {
      background = "#a3313b";
      text = QString("과속 감지 · 제한 60 km/h\n현재 %1  |  최고 %2 km/h")
               .arg(status.kmh, 0, 'f', 2)
               .arg(status.peak_kmh, 0, 'f', 2);
    }
  }
  speed_label_->setText(text);
  const auto style = QString(
                       "QLabel { background: %1; color: %2; border-radius: 6px; "
                       "padding: 6px; font-size: 13px; font-weight: 600; }")
                       .arg(background, foreground);
  if (speed_label_->styleSheet() != style) speed_label_->setStyleSheet(style);
}

void GoalSetterPanel::start()
{
  if (!button_->isEnabled()) return;
  if (
    odom_->header.frame_id != "map" || kcity::distance(odom_->pose.pose, kcity::pose(0)) > 150.0) {
    button_->setToolTip("Initialize localization near START/END before setting the course");
    return;
  }
  next_ = 1;
  final_leg_ = false;
  active_ = true;
  progress_.reset();
  tracking_progress_ = true;
  showProgress();
  showMarkers();
  send(false);
}

void GoalSetterPanel::send(bool final_leg)
{
  auto req = std::make_shared<Service::Request>(kcity::request(final_leg, next_));
  req->header.stamp = node_->now();
  pending_ = true;
  timed_out_ = false;
  sent_at_ = last_attempt_ = Clock::now();
  button_->setToolTip(
    final_leg ? "Updating final goal automatically..." : "Setting checkpoints 1-14...");
  const auto callback = [this, final_leg](rclcpp::Client<Service>::SharedFuture future) {
    pending_ = false;
    const auto result = future.get()->status;
    if (!result.success) {
      if (!final_leg) active_ = false;
      button_->setToolTip(
        QString("%1: %2 (%3)")
          .arg(final_leg ? "Update rejected; keeping route, will retry" : "Route setup failed")
          .arg(QString::fromStdString(result.message))
          .arg(result.code));
      return;
    }
    final_leg_ = final_leg;
    button_->setToolTip(
      final_leg ? "Stop goal: 60 m beyond END. END scoring unchanged."
                : "Ready. Press Auto once. Final goal will update after checkpoint 2.");
  };
  (final_leg ? change_ : set_)->async_send_request(req, callback);
}

void GoalSetterPanel::stop(const QString & reason)
{
  active_ = false;
  button_->setToolTip(reason);
}

void GoalSetterPanel::showProgress()
{
  count_->setText(QString("성공 %1 / 15 (CP 1–14 + END)").arg(progress_.successes()));
  for (std::size_t i = 0; i < lights_.size(); ++i) {
    const auto result = progress_.results[i];
    const auto color = result == kcity::CheckpointResult::Success  ? "#22c55e"
                       : result == kcity::CheckpointResult::Missed ? "#ef4444"
                                                                   : "#737373";
    const auto name = i == 0 ? QString("END") : QString::number(i);
    lights_[i]->setText(name);
    lights_[i]->setStyleSheet(QString(
                                "background-color: %1; color: white; border-radius: 19px; "
                                "font-size: 12px; font-weight: bold;")
                                .arg(color));
    lights_[i]->setToolTip(
      name + (result == kcity::CheckpointResult::Success  ? ": 성공"
              : result == kcity::CheckpointResult::Missed ? ": 놓침"
                                                          : ": 미도달"));
  }
}

void GoalSetterPanel::showMarkers()
{
  if (!map_ok_) return;
  visualization_msgs::msg::MarkerArray array;
  for (std::size_t i = 0; i < kcity::course.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node_->now();
    marker.ns = "kcity_course";
    marker.id = static_cast<int>(i);
    marker.type = marker.ARROW;
    marker.action = marker.ADD;
    marker.pose = kcity::pose(i);
    marker.scale.x = 4.0;
    marker.scale.y = .5;
    marker.scale.z = .5;
    marker.color.a = 1.0;
    marker.color.r = i == 0 ? 1.0 : .2;
    marker.color.g = i > 0 && i < next_ ? .8 : .4;
    marker.color.b = i == 0 ? .1 : .9;
    array.markers.push_back(marker);
    marker.id += 100;
    marker.type = marker.TEXT_VIEW_FACING;
    marker.pose.position.z += 3.0;
    marker.scale.z = 2.0;
    marker.text = i == 0 ? "START / END" : std::to_string(i);
    array.markers.push_back(marker);
  }
  auto stop_goal = array.markers.front();
  stop_goal.id = 200;
  stop_goal.pose = kcity::stop_goal_pose();
  stop_goal.color.r = 1.0;
  stop_goal.color.g = 0.5;
  stop_goal.color.b = 0.0;
  array.markers.push_back(stop_goal);
  stop_goal.id = 201;
  stop_goal.type = stop_goal.TEXT_VIEW_FACING;
  stop_goal.pose.position.z += 3.0;
  stop_goal.scale.z = 2.0;
  stop_goal.text = "STOP GOAL";
  array.markers.push_back(stop_goal);
  markers_->publish(array);
}
}  // namespace tier4_adapi_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tier4_adapi_rviz_plugins::GoalSetterPanel, rviz_common::Panel)
