// Copyright 2025 Tier IV, Inc.
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

#include "colored_pose_with_covariance_history_display.hpp"

#include "value_overlay.hpp"

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <limits>
#include <memory>

namespace rviz_plugins
{
ColoredPoseWithCovarianceHistory::ColoredPoseWithCovarianceHistory()
: last_stamp_(0, 0, RCL_ROS_TIME)
{
  property_value_type_ = new rviz_common::properties::EnumProperty(
    "Value Type", "Float32", "Select the type of value to visualize", this,
    SLOT(update_value_type()));
  property_value_type_->addOption("Int32", static_cast<int>(ValueType::Int32));
  property_value_type_->addOption("Float32", static_cast<int>(ValueType::Float32));

  property_value_topic_ = new rviz_common::properties::RosTopicProperty(
    "Value Topic", "",
    rosidl_generator_traits::name<autoware_internal_debug_msgs::msg::Float32Stamped>(), "", this,
    SLOT(update_value_topic()));

  property_buffer_size_ = new rviz_common::properties::IntProperty("Buffer Size", 100, "", this);
  property_buffer_size_->setMin(0);
  property_buffer_size_->setMax(100000);

  property_line_view_ = new rviz_common::properties::BoolProperty("Path", true, "", this);

  property_line_width_ =
    new rviz_common::properties::FloatProperty("Width", 0.1, "", property_line_view_);
  property_line_width_->setMin(0.0);

  property_line_alpha_ =
    new rviz_common::properties::FloatProperty("Alpha", 1.0, "", property_line_view_);
  property_line_alpha_->setMin(0.0);
  property_line_alpha_->setMax(1.0);

  property_line_min_color_ =
    new rviz_common::properties::ColorProperty("Min Color", Qt::blue, "", property_line_view_);

  property_line_max_color_ =
    new rviz_common::properties::ColorProperty("Max Color", Qt::red, "", property_line_view_);

  property_line_less_color_ =
    new rviz_common::properties::ColorProperty("Less Color", Qt::blue, "", property_line_view_);

  property_line_greater_color_ =
    new rviz_common::properties::ColorProperty("Greater Color", Qt::red, "", property_line_view_);

  property_auto_min_max_ =
    new rviz_common::properties::BoolProperty("Auto Set Min/Max", true, "", this);

  property_min_value_ =
    new rviz_common::properties::FloatProperty("Min Value", 0.0, "", property_auto_min_max_);

  property_max_value_ =
    new rviz_common::properties::FloatProperty("Max Value", 1.0, "", property_auto_min_max_);
  property_overlay_ = new rviz_common::properties::BoolProperty(
    "Value Overlay", false, "Show latest value at the top left, using the path color scale", this);
  property_overlay_label_ = new rviz_common::properties::StringProperty(
    "Label", "Value", "Overlay title", property_overlay_);
}

ColoredPoseWithCovarianceHistory::~ColoredPoseWithCovarianceHistory()
{
  delete property_value_type_;
}

void ColoredPoseWithCovarianceHistory::onInitialize()
{
  MFDClass::onInitialize();
  lines_ = std::make_unique<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
  auto node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    return;
  }

  node_ = node_abstraction->get_raw_node();
  if (!node_) {
    return;
  }

  property_value_topic_->initialize(node_abstraction);
  overlay_ = std::make_unique<autoware_overlay_rviz_plugin::OverlayObject>(
    "localization_value_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)));
  overlay_->updateTextureSize(200, 112);
  overlay_->setDimensions(200, 112);
  overlay_->setPosition(12, 12);
  overlay_->hide();
}

void ColoredPoseWithCovarianceHistory::onEnable()
{
  subscribe();
}

void ColoredPoseWithCovarianceHistory::onDisable()
{
  unsubscribe();
  if (overlay_) overlay_->hide();
}

void ColoredPoseWithCovarianceHistory::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  if (!history_.empty()) {
    lines_->clear();
    if (property_line_view_->getBool()) {
      update_visualization();
    }
  }
  update_overlay();
}

void ColoredPoseWithCovarianceHistory::update_overlay()
{
  if (!overlay_) return;
  if (!property_overlay_->getBool()) {
    overlay_->hide();
    return;
  }
  overlay_->show();
  const bool fresh = std::isfinite(last_value_) &&
                     std::chrono::steady_clock::now() - value_received_ < std::chrono::seconds(1);
  const auto value_color =
    fresh ? get_color_from_value(last_value_) : Ogre::ColourValue(0.5, 0.5, 0.5);
  auto buffer = overlay_->getBuffer();
  auto image = buffer.getQImage(*overlay_);
  image.fill(Qt::transparent);
  QPainter painter(&image);
  paint_value_overlay(
    painter, property_overlay_label_->getString(), last_value_, fresh,
    QColor::fromRgbF(value_color.r, value_color.g, value_color.b), property_min_value_->getFloat(),
    property_max_value_->getFloat(), property_line_min_color_->getColor(),
    property_line_max_color_->getColor());
}

void ColoredPoseWithCovarianceHistory::update_value_type()
{
  current_value_type_ = static_cast<ValueType>(property_value_type_->getOptionInt());

  if (current_value_type_ == ValueType::Int32) {
    property_value_topic_->setMessageType(
      rosidl_generator_traits::name<autoware_internal_debug_msgs::msg::Int32Stamped>());
  } else {
    property_value_topic_->setMessageType(
      rosidl_generator_traits::name<autoware_internal_debug_msgs::msg::Float32Stamped>());
  }

  update_value_topic();
}

void ColoredPoseWithCovarianceHistory::update_value_topic()
{
  lines_->clear();
  int32_sub_.reset();
  float32_sub_.reset();
  last_value_ = std::numeric_limits<double>::quiet_NaN();
  if (property_value_topic_->getStdString().empty()) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Value Topic", "No topic was set.");
    return;
  }
  if (current_value_type_ == ValueType::Int32) {
    int32_sub_ = node_->create_subscription<autoware_internal_debug_msgs::msg::Int32Stamped>(
      property_value_topic_->getStdString(), rclcpp::QoS(10),
      std::bind(
        &ColoredPoseWithCovarianceHistory::process_int32_message, this, std::placeholders::_1));
  } else {
    float32_sub_ = node_->create_subscription<autoware_internal_debug_msgs::msg::Float32Stamped>(
      property_value_topic_->getStdString(), rclcpp::QoS(10),
      std::bind(
        &ColoredPoseWithCovarianceHistory::process_float32_message, this, std::placeholders::_1));
  }
  setStatus(rviz_common::properties::StatusProperty::Ok, "Value Topic", "OK");
}

void ColoredPoseWithCovarianceHistory::update_visualization()
{
  if (history_.empty()) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", "No messages received yet");
    return;
  }
  if (property_auto_min_max_->getBool()) {
    double min_value = std::numeric_limits<double>::max();
    double max_value = std::numeric_limits<double>::min();
    for (const auto & pose_with_value : history_) {
      if (pose_with_value.value < min_value) {
        min_value = pose_with_value.value;
      }
      if (pose_with_value.value > max_value) {
        max_value = pose_with_value.value;
      }
    }
    property_min_value_->setFloat(min_value);
    property_max_value_->setFloat(max_value);
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  auto frame_manager = context_->getFrameManager();
  if (!frame_manager->getTransform(target_frame_, last_stamp_, position, orientation)) {
    setMissingTransformToFixedFrame(target_frame_);
    return;
  }

  setTransformOk();

  lines_->clear();
  lines_->setMaxPointsPerLine(history_.size());
  lines_->setLineWidth(property_line_width_->getFloat());
  lines_->setPosition(position);
  lines_->setOrientation(orientation);

  for (const auto & pose_with_value : history_) {
    const auto color = get_color_from_value(pose_with_value.value);
    Ogre::Vector3 point;
    point.x = static_cast<float>(pose_with_value.pose->pose.pose.position.x);
    point.y = static_cast<float>(pose_with_value.pose->pose.pose.position.y);
    point.z = static_cast<float>(pose_with_value.pose->pose.pose.position.z);
    lines_->addPoint(point, color);
  }
}

void ColoredPoseWithCovarianceHistory::subscribe()
{
  history_.clear();
  lines_->clear();

  MFDClass::subscribe();
  update_value_topic();
}

void ColoredPoseWithCovarianceHistory::unsubscribe()
{
  MFDClass::unsubscribe();
  int32_sub_.reset();
  float32_sub_.reset();

  history_.clear();
  lines_->clear();
}

void ColoredPoseWithCovarianceHistory::processMessage(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message)
{
  if (!std::isfinite(last_value_)) return;
  if (
    !rviz_common::validateFloats(message->pose.pose) ||
    !rviz_common::validateFloats(message->pose.covariance)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }
  if (target_frame_ != message->header.frame_id) {
    history_.clear();
    target_frame_ = message->header.frame_id;
  }
  history_.emplace_back(message, last_value_);
  last_stamp_ = message->header.stamp;
  update_history();
  update_visualization();
}

void ColoredPoseWithCovarianceHistory::process_int32_message(
  const autoware_internal_debug_msgs::msg::Int32Stamped::ConstSharedPtr message)
{
  last_value_ = static_cast<double>(message->data);
  value_received_ = std::chrono::steady_clock::now();
}

void ColoredPoseWithCovarianceHistory::process_float32_message(
  const autoware_internal_debug_msgs::msg::Float32Stamped::ConstSharedPtr message)
{
  last_value_ = static_cast<double>(message->data);
  value_received_ = std::chrono::steady_clock::now();
}

void ColoredPoseWithCovarianceHistory::update_history()
{
  const auto buffer_size = static_cast<size_t>(property_buffer_size_->getInt());
  while (buffer_size < history_.size()) {
    history_.pop_front();
  }
}

Ogre::ColourValue ColoredPoseWithCovarianceHistory::get_color_from_value(double value)
{
  const auto min_value = property_min_value_->getFloat();
  const auto max_value = property_max_value_->getFloat();
  if (value < min_value) {
    return property_line_less_color_->getOgreColor();
  }
  if (value > max_value) {
    return property_line_greater_color_->getOgreColor();
  }
  if (min_value == max_value) {
    return property_line_min_color_->getOgreColor();  // Avoid division by zero
  }
  const auto ratio = (value - min_value) / (max_value - min_value);
  const auto min_color = property_line_min_color_->getOgreColor();
  const auto max_color = property_line_max_color_->getOgreColor();
  const auto color =
    min_color * (1.0f - static_cast<float>(ratio)) + max_color * static_cast<float>(ratio);
  return color;
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ColoredPoseWithCovarianceHistory, rviz_common::Display)
