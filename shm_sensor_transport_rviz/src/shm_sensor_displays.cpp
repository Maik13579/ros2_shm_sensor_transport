// Copyright 2026 Maik Knof
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

#include "shm_sensor_transport_rviz/shm_image_display.hpp"
#include "shm_sensor_transport_rviz/shm_point_cloud2_display.hpp"

#include <memory>
#include <string>
#include <utility>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <shm_sensor_transport_interfaces/msg/shm_image.hpp>
#include <shm_sensor_transport_interfaces/msg/shm_point_cloud2.hpp>

#include "shm_sensor_transport/qos_utils.hpp"

namespace shm_sensor_transport_rviz
{
namespace
{

template<typename MetadataMessageT>
QString metadataMessageType()
{
  return QString::fromStdString(rosidl_generator_traits::name<MetadataMessageT>());
}

void setStatusError(rviz_common::Display & display, const char * message)
{
  display.setStatus(
    rviz_common::properties::StatusProperty::Error,
    "Topic",
    QString("Error subscribing: ") + message);
}

void setShmDefaultQos(rviz_common::properties::RosTopicProperty * topic_property)
{
  topic_property->subProp("Depth")->setValue(1);
  topic_property->subProp("History Policy")->setValue("Keep Last");
  topic_property->subProp("Reliability Policy")->setValue("Best Effort");
  topic_property->subProp("Durability Policy")->setValue("Volatile");
}

}  // namespace

ShmImageDisplay::ShmImageDisplay()
{
  qos_profile = shm_sensor_transport::make_metadata_qos(true, false, 1);
  const auto message_type =
    metadataMessageType<shm_sensor_transport_interfaces::msg::ShmImage>();
  topic_property_->setMessageType(message_type);
  topic_property_->setDescription(message_type + " topic to subscribe to.");
  setShmDefaultQos(topic_property_);
}

ShmImageDisplay::~ShmImageDisplay()
{
  unsubscribe();
}

void ShmImageDisplay::subscribe()
{
  if (!isEnabled()) {
    return;
  }

  if (topic_property_->isEmpty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      QString("Error subscribing: Empty topic name"));
    return;
  }

  try {
    const auto node = rviz_ros_node_.lock()->get_raw_node();
    shm_sensor_transport::ShmSubscriberOptions options;
    options.qos = qos_profile;
    shm_subscription_ = std::make_shared<shm_sensor_transport::ShmImageSubscriber>(
      node.get(),
      topic_property_->getTopicStd(),
      [this](
        std::unique_ptr<sensor_msgs::msg::Image> message,
        const shm_sensor_transport_interfaces::msg::ShmImage &) {
        std::shared_ptr<sensor_msgs::msg::Image> shared_message = std::move(message);
        processMessage(shared_message);
      },
      options);
    subscription_start_time_ = node->now();
    setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
  } catch (const rclcpp::exceptions::InvalidTopicNameError & error) {
    setStatusError(*this, error.what());
  } catch (const std::exception & error) {
    setStatusError(*this, error.what());
  }
}

void ShmImageDisplay::unsubscribe()
{
  if (shm_subscription_) {
    shm_subscription_->close();
  }
  shm_subscription_.reset();
}

ShmPointCloud2Display::ShmPointCloud2Display()
{
  qos_profile = shm_sensor_transport::make_metadata_qos(true, false, 1);
  const auto message_type =
    metadataMessageType<shm_sensor_transport_interfaces::msg::ShmPointCloud2>();
  topic_property_->setMessageType(message_type);
  topic_property_->setDescription(message_type + " topic to subscribe to.");
  setShmDefaultQos(topic_property_);
}

ShmPointCloud2Display::~ShmPointCloud2Display()
{
  unsubscribe();
}

void ShmPointCloud2Display::subscribe()
{
  if (!isEnabled()) {
    return;
  }

  if (topic_property_->isEmpty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      QString("Error subscribing: Empty topic name"));
    return;
  }

  try {
    const auto node = rviz_ros_node_.lock()->get_raw_node();
    shm_sensor_transport::ShmSubscriberOptions options;
    options.qos = qos_profile;
    shm_subscription_ = std::make_shared<shm_sensor_transport::ShmPointCloud2Subscriber>(
      node.get(),
      topic_property_->getTopicStd(),
      [this](
        std::unique_ptr<sensor_msgs::msg::PointCloud2> message,
        const shm_sensor_transport_interfaces::msg::ShmPointCloud2 &) {
        std::shared_ptr<sensor_msgs::msg::PointCloud2> shared_message = std::move(message);
        processMessage(shared_message);
      },
      options);
    subscription_start_time_ = node->now();
    setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
  } catch (const rclcpp::exceptions::InvalidTopicNameError & error) {
    setStatusError(*this, error.what());
  } catch (const std::exception & error) {
    setStatusError(*this, error.what());
  }
}

void ShmPointCloud2Display::unsubscribe()
{
  if (shm_subscription_) {
    shm_subscription_->close();
  }
  shm_subscription_.reset();
}

}  // namespace shm_sensor_transport_rviz

PLUGINLIB_EXPORT_CLASS(
  shm_sensor_transport_rviz::ShmImageDisplay,
  rviz_common::Display)

PLUGINLIB_EXPORT_CLASS(
  shm_sensor_transport_rviz::ShmPointCloud2Display,
  rviz_common::Display)
