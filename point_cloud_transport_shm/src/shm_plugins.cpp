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
#include <optional>
#include <string>
#include <utility>

#include <pluginlib/class_list_macros.hpp>
#include <point_cloud_transport/publisher_plugin.hpp>
#include <point_cloud_transport/subscriber_plugin.hpp>
#if __has_include(<point_cloud_transport/version.h>)
#include <point_cloud_transport/version.h>
#endif
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "shm_sensor_transport/shm_publisher.hpp"
#include "shm_sensor_transport/shm_subscriber.hpp"

#if defined(POINT_CLOUD_TRANSPORT_VERSION_MAJOR) && POINT_CLOUD_TRANSPORT_VERSION_MAJOR >= 4
#define POINT_CLOUD_TRANSPORT_SHM_HAS_PUBLISH_PTR 1
#else
#define POINT_CLOUD_TRANSPORT_SHM_HAS_PUBLISH_PTR 0
#endif

namespace point_cloud_transport_shm
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

class ShmPublisherPlugin : public point_cloud_transport::PublisherPlugin
{
public:
  std::string getTransportName() const override
  {
    return "shm";
  }

  uint32_t getNumSubscribers() const override
  {
    if (!publisher_) {
      return 0U;
    }
    return static_cast<uint32_t>(publisher_->metadata_publisher()->get_subscription_count());
  }

  std::string getTopic() const override
  {
    return publisher_ ? publisher_->metadata_topic() : std::string{};
  }

  std::string getDataType() const override
  {
    return "shm_sensor_transport_interfaces/msg/ShmPointCloud2";
  }

  EncodeResult encode(const sensor_msgs::msg::PointCloud2 & /*raw*/) const override
  {
    return std::nullopt;
  }

  void publish(const sensor_msgs::msg::PointCloud2 & message) const override
  {
    if (publisher_) {
      publisher_->publish(message);
    }
  }

#if POINT_CLOUD_TRANSPORT_SHM_HAS_PUBLISH_PTR
  void publishPtr(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message) const override
  {
    if (message) {
      publish(*message);
    }
  }
#endif

  rclcpp::PublisherBase::SharedPtr getPublisher() const override
  {
    return publisher_ ? publisher_->metadata_publisher() : nullptr;
  }

  void shutdown() override
  {
    publisher_.reset();
    node_.reset();
  }

  void declareParameters(const std::string & /*base_topic*/) override
  {
  }

  std::string getTopicToAdvertise(const std::string & base_topic) const override
  {
    return shm_sensor_transport::resolve_metadata_topic(base_topic);
  }

protected:
  void advertiseImpl(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & base_topic,
    rmw_qos_profile_t custom_qos,
    const rclcpp::PublisherOptions & /*options*/) override
  {
    node_ = std::move(node);
    publisher_ = std::make_unique<shm_sensor_transport::ShmPointCloud2Publisher>(
      node_.get(), base_topic, publisher_options(*node_, base_topic, custom_qos));
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<shm_sensor_transport::ShmPointCloud2Publisher> publisher_;
};

class ShmSubscriberPlugin : public point_cloud_transport::SubscriberPlugin
{
public:
  std::string getTransportName() const override
  {
    return "shm";
  }

  DecodeResult decode(const std::shared_ptr<rclcpp::SerializedMessage> & /*compressed*/)
  const override
  {
    return std::nullopt;
  }

  rclcpp::SubscriptionBase::SharedPtr getSubscription() const override
  {
    return subscriber_ ? subscriber_->subscription() : nullptr;
  }

  std::string getTopic() const override
  {
    return subscriber_ ? subscriber_->metadata_topic() : std::string{};
  }

  uint32_t getNumPublishers() const override
  {
    if (!subscriber_) {
      return 0U;
    }
    return static_cast<uint32_t>(subscriber_->subscription()->get_publisher_count());
  }

  void shutdown() override
  {
    subscriber_.reset();
    node_.reset();
  }

  std::string getDataType() const override
  {
    return "shm_sensor_transport_interfaces/msg/ShmPointCloud2";
  }

  void declareParameters() override
  {
  }

  std::string getTopicToSubscribe(const std::string & base_topic) const override
  {
    return shm_sensor_transport::resolve_metadata_topic(base_topic);
  }

protected:
  void subscribeImpl(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos) override
  {
    subscribeImpl(node, base_topic, callback, custom_qos, rclcpp::SubscriptionOptions{});
  }

  void subscribeImpl(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos,
    rclcpp::SubscriptionOptions /*options*/) override
  {
    node_ = std::move(node);
    subscriber_ = std::make_unique<shm_sensor_transport::ShmPointCloud2Subscriber>(
      node_.get(),
      base_topic,
      [callback](
        sensor_msgs::msg::PointCloud2::UniquePtr msg,
        const shm_sensor_transport_interfaces::msg::ShmPointCloud2 &)
      {
        callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr(std::move(msg)));
      },
      subscriber_options(*node_, base_topic, custom_qos));
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<shm_sensor_transport::ShmPointCloud2Subscriber> subscriber_;
};

}  // namespace point_cloud_transport_shm

PLUGINLIB_EXPORT_CLASS(
  point_cloud_transport_shm::ShmPublisherPlugin, point_cloud_transport::PublisherPlugin)
PLUGINLIB_EXPORT_CLASS(
  point_cloud_transport_shm::ShmSubscriberPlugin, point_cloud_transport::SubscriberPlugin)
