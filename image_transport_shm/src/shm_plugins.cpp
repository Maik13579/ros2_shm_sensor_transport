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

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include <image_transport/publisher_plugin.hpp>
#include <image_transport/subscriber_plugin.hpp>
#if __has_include(<image_transport/version.h>)
#include <image_transport/version.h>
#endif
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "shm_sensor_transport/shm_publisher.hpp"
#include "shm_sensor_transport/shm_subscriber.hpp"

#if defined(IMAGE_TRANSPORT_VERSION_MAJOR) && IMAGE_TRANSPORT_VERSION_MAJOR >= 5
#define IMAGE_TRANSPORT_SHM_HAS_PUBLISHER_OPTIONS_OVERRIDE 1
#else
#define IMAGE_TRANSPORT_SHM_HAS_PUBLISHER_OPTIONS_OVERRIDE 0
#endif

namespace image_transport_shm
{

namespace
{

rclcpp::QoS make_qos(const rmw_qos_profile_t & custom_qos)
{
  return rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos), custom_qos);
}

std::string parameter_prefix(const std::string & base_topic)
{
  auto prefix = shm_sensor_transport::resolve_metadata_topic(base_topic);
  if (!prefix.empty() && prefix.front() == '/') {
    prefix.erase(prefix.begin());
  }
  std::replace(prefix.begin(), prefix.end(), '/', '.');
  return prefix;
}

template<typename T>
T declare_or_get_parameter(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  try {
    return node.declare_parameter<T>(name, default_value);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    return node.get_parameter(name).get_value<T>();
  }
}

shm_sensor_transport::ShmPublisherOptions publisher_options(
  rclcpp::Node & node,
  const std::string & base_topic,
  const rmw_qos_profile_t & custom_qos)
{
  const auto prefix = parameter_prefix(base_topic);
  shm_sensor_transport::ShmPublisherOptions options;
  options.qos = make_qos(custom_qos);
  options.shm_name = declare_or_get_parameter<std::string>(node, prefix + ".shm_name", "");
  options.slot_count = declare_or_get_parameter<int>(node, prefix + ".slot_count", 8);
  const auto slot_size = declare_or_get_parameter<std::int64_t>(
    node, prefix + ".slot_size_bytes", 0);
  options.slot_size_bytes = static_cast<std::uint64_t>(std::max<std::int64_t>(0, slot_size));
  options.allow_resize = declare_or_get_parameter<bool>(node, prefix + ".allow_resize", false);
  options.warn_on_oversized_frame = declare_or_get_parameter<bool>(
    node, prefix + ".warn_on_oversized_frame", true);
  return options;
}

shm_sensor_transport::ShmSubscriberOptions subscriber_options(
  rclcpp::Node & node,
  const std::string & base_topic,
  const rmw_qos_profile_t & custom_qos)
{
  const auto prefix = parameter_prefix(base_topic);
  shm_sensor_transport::ShmSubscriberOptions options;
  options.qos = make_qos(custom_qos);
  options.rate_limit_hz = declare_or_get_parameter<double>(node, prefix + ".rate_limit_hz", 0.0);
  return options;
}

}  // namespace

class ShmPublisherPlugin : public image_transport::PublisherPlugin
{
public:
  std::string getTransportName() const override
  {
    return "shm";
  }

  size_t getNumSubscribers() const override
  {
    return publisher_ ? publisher_->metadata_publisher()->get_subscription_count() : 0U;
  }

  std::string getTopic() const override
  {
    return publisher_ ? publisher_->metadata_topic() : std::string{};
  }

  void publish(const sensor_msgs::msg::Image & message) const override
  {
    if (publisher_) {
      publisher_->publish(message);
    }
  }

  void publishPtr(const sensor_msgs::msg::Image::ConstSharedPtr & message) const override
  {
    if (message) {
      publish(*message);
    }
  }

  void shutdown() override
  {
    publisher_.reset();
  }

protected:
#if IMAGE_TRANSPORT_SHM_HAS_PUBLISHER_OPTIONS_OVERRIDE
  void advertiseImpl(
    rclcpp::Node * node,
    const std::string & base_topic,
    rmw_qos_profile_t custom_qos,
    rclcpp::PublisherOptions /*options*/) override
#else
  void advertiseImpl(
    rclcpp::Node * node,
    const std::string & base_topic,
    rmw_qos_profile_t custom_qos) override
#endif
  {
    advertise_shm(node, base_topic, custom_qos);
  }

private:
  void advertise_shm(
    rclcpp::Node * node,
    const std::string & base_topic,
    rmw_qos_profile_t custom_qos)
  {
    publisher_ = std::make_unique<shm_sensor_transport::ShmImagePublisher>(
      node, base_topic, publisher_options(*node, base_topic, custom_qos));
  }

  std::unique_ptr<shm_sensor_transport::ShmImagePublisher> publisher_;
};

class ShmSubscriberPlugin : public image_transport::SubscriberPlugin
{
public:
  std::string getTransportName() const override
  {
    return "shm";
  }

  std::string getTopic() const override
  {
    return subscriber_ ? subscriber_->metadata_topic() : std::string{};
  }

  size_t getNumPublishers() const override
  {
    return subscriber_ ? subscriber_->subscription()->get_publisher_count() : 0U;
  }

  void shutdown() override
  {
    subscriber_.reset();
  }

protected:
  void subscribeImpl(
    rclcpp::Node * node,
    const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos,
    rclcpp::SubscriptionOptions /*options*/) override
  {
    subscribe_shm(node, base_topic, callback, custom_qos);
  }

#if !IMAGE_TRANSPORT_SHM_HAS_PUBLISHER_OPTIONS_OVERRIDE
  void subscribeImpl(
    rclcpp::Node * node,
    const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos) override
  {
    subscribe_shm(node, base_topic, callback, custom_qos);
  }
#endif

private:
  void subscribe_shm(
    rclcpp::Node * node,
    const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos)
  {
    subscriber_ = std::make_unique<shm_sensor_transport::ShmImageSubscriber>(
      node,
      base_topic,
      [callback](
        sensor_msgs::msg::Image::UniquePtr msg,
        const shm_sensor_transport_interfaces::msg::ShmImage &)
      {
        callback(sensor_msgs::msg::Image::ConstSharedPtr(std::move(msg)));
      },
      subscriber_options(*node, base_topic, custom_qos));
  }

  std::unique_ptr<shm_sensor_transport::ShmImageSubscriber> subscriber_;
};

}  // namespace image_transport_shm

PLUGINLIB_EXPORT_CLASS(image_transport_shm::ShmPublisherPlugin, image_transport::PublisherPlugin)
PLUGINLIB_EXPORT_CLASS(image_transport_shm::ShmSubscriberPlugin, image_transport::SubscriberPlugin)
