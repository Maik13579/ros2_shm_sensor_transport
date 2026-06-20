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

#ifndef ADAPTED_IMAGE_TRANSPORT__ADAPTED_IMAGE_TRANSPORT_HPP_
#define ADAPTED_IMAGE_TRANSPORT__ADAPTED_IMAGE_TRANSPORT_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <typeindex>
#include <utility>
#include <vector>

#include "image_transport/image_transport.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/intra_process_setting.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace adapted_image_transport
{
namespace detail
{

template<typename AdapterT>
using RosMessageT = typename rclcpp::TypeAdapter<AdapterT>::ros_message_type;

template<typename AdapterT>
constexpr bool is_valid_adapter_v =
  rclcpp::TypeAdapter<AdapterT>::is_specialized::value &&
  std::is_same<RosMessageT<AdapterT>, sensor_msgs::msg::Image>::value;

inline rclcpp::QoS qos_from_rmw(const rmw_qos_profile_t & profile)
{
  return rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(profile), profile);
}

template<typename OptionsT>
auto create_public_publisher(
  rclcpp::Node * node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos,
  const OptionsT & options,
  int)
-> decltype(image_transport::create_publisher(node, base_topic, custom_qos, options))
{
  return image_transport::create_publisher(node, base_topic, custom_qos, options);
}

template<typename OptionsT>
image_transport::Publisher create_public_publisher(
  rclcpp::Node * node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos,
  const OptionsT &,
  double)
{
  return image_transport::create_publisher(node, base_topic, custom_qos);
}

/// Context-local registry for adapted publishers and same-process subscribers.
class Registry
{
public:
  /// Description of one private adapted publisher endpoint.
  struct PublisherInfo
  {
    std::string base_topic;
    std::type_index adapter_type{typeid(void)};
    std::string adapted_topic;
    uint64_t token = 0;
  };

  /// Allocate a process-local token for a private adapted publisher topic.
  uint64_t next_token()
  {
    return next_token_++;
  }

  /// Register a private adapted topic and notify matching subscribers.
  void register_publisher(
    const std::string & base_topic,
    std::type_index adapter_type,
    const std::string & adapted_topic,
    uint64_t token)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      publishers_[token] = PublisherInfo{base_topic, adapter_type, adapted_topic, token};
    }
    // Wake subscribers that are currently on the public fallback path.
    trigger_matching(base_topic, adapter_type);
  }

  /// Unregister a private adapted topic and notify matching subscribers.
  void unregister_publisher(uint64_t token)
  {
    PublisherInfo removed;
    bool found = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto it = publishers_.find(token);
      if (it != publishers_.end()) {
        removed = it->second;
        publishers_.erase(it);
        found = true;
      }
    }
    if (found) {
      // Wake subscribers so they can fall back to the public transport path.
      trigger_matching(removed.base_topic, removed.adapter_type);
    }
  }

  /// Register subscriber interest and the guard condition to wake on changes.
  uint64_t add_interest(
    const std::string & base_topic,
    std::type_index adapter_type,
    rclcpp::GuardCondition::WeakPtr guard_condition)
  {
    const uint64_t id = next_interest_++;
    std::lock_guard<std::mutex> lock(mutex_);
    interests_[id] = Interest{base_topic, adapter_type, std::move(guard_condition)};
    return id;
  }

  /// Remove subscriber interest when the subscriber shuts down.
  void remove_interest(uint64_t id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    interests_.erase(id);
  }

  /// Return matching adapted publishers for a base topic and adapter type.
  std::vector<PublisherInfo> find_publishers(
    const std::string & base_topic,
    std::type_index adapter_type) const
  {
    std::vector<PublisherInfo> matches;
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & item : publishers_) {
      const auto & publisher = item.second;
      if (publisher.base_topic == base_topic && publisher.adapter_type == adapter_type) {
        matches.push_back(publisher);
      }
    }
    return matches;
  }

  /// Count live same-process subscribers interested in an adapted stream.
  size_t local_subscriber_count(
    const std::string & base_topic,
    std::type_index adapter_type) const
  {
    size_t count = 0;
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & item : interests_) {
      const auto & interest = item.second;
      if (interest.base_topic == base_topic && interest.adapter_type == adapter_type &&
        !interest.guard_condition.expired())
      {
        ++count;
      }
    }
    return count;
  }

private:
  struct Interest
  {
    std::string base_topic;
    std::type_index adapter_type{typeid(void)};
    rclcpp::GuardCondition::WeakPtr guard_condition;
  };

  void trigger_matching(
    const std::string & base_topic,
    std::type_index adapter_type) const
  {
    std::vector<rclcpp::GuardCondition::SharedPtr> guards;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & item : interests_) {
        const auto & interest = item.second;
        if (interest.base_topic == base_topic && interest.adapter_type == adapter_type) {
          if (auto guard = interest.guard_condition.lock()) {
            guards.push_back(std::move(guard));
          }
        }
      }
    }
    for (const auto & guard : guards) {
      guard->trigger();
    }
  }

  mutable std::mutex mutex_;
  std::atomic_uint64_t next_token_{1};
  std::atomic_uint64_t next_interest_{1};
  std::map<uint64_t, PublisherInfo> publishers_;
  std::map<uint64_t, Interest> interests_;
};

/// Return the registry scoped to the node's rclcpp context.
inline std::shared_ptr<Registry> registry_for(const rclcpp::Node::SharedPtr & node)
{
  return node->get_node_base_interface()->get_context()->get_sub_context<Registry>();
}

/// Build the private implementation topic for a registered adapted publisher.
inline std::string adapted_topic_name(const std::string & base_topic, uint64_t token)
{
  return base_topic + "/_adapted/pub_" + std::to_string(token);
}

}  // namespace detail

template<typename AdapterT>
/// Type-adapted image publisher with public image_transport compatibility.
class Publisher
{
  static_assert(
    detail::is_valid_adapter_v<AdapterT>,
    "adapted_image_transport::Publisher requires an rclcpp TypeAdapter whose ros_message_type "
    "is sensor_msgs::msg::Image");

public:
  using Adapter = rclcpp::TypeAdapter<AdapterT>;
  using CustomMessage = typename Adapter::custom_type;
  using RosMessage = typename Adapter::ros_message_type;

  Publisher() = default;

  /// Create a publisher for the public base topic and private adapted topic.
  /**
   * \param node Node used for both public and private publishers.
   * \param base_topic Public image topic. Transport plugins derive subtopics from it.
   * \param custom_qos QoS profile passed to public and private publishers.
   * \param public_options Options passed to the public image_transport publisher.
   * \param async_publish When true, publish() queues the latest message and returns
   *   before type conversion or transport publishing runs in the executor.
   */
  Publisher(
    rclcpp::Node::SharedPtr node,
    std::string base_topic,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
    rclcpp::PublisherOptions public_options = rclcpp::PublisherOptions(),
    bool async_publish = true)
  : node_(std::move(node)),
    base_topic_(std::move(base_topic)),
    async_publish_(async_publish),
    registry_(detail::registry_for(node_)),
    token_(registry_->next_token()),
    adapted_topic_(detail::adapted_topic_name(base_topic_, token_)),
    public_publisher_(detail::create_public_publisher(
        node_.get(), base_topic_, custom_qos, public_options, 0))
  {
    auto adapted_options = public_options;
    adapted_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    adapted_publisher_ = node_->create_publisher<AdapterT>(
      adapted_topic_, detail::qos_from_rmw(custom_qos), adapted_options);
    publish_timer_ = rclcpp::create_wall_timer(
      std::chrono::nanoseconds(1),
      [this]() {drain_pending_publish();},
      nullptr,
      node_->get_node_base_interface().get(),
      node_->get_node_timers_interface().get());
    publish_timer_->cancel();
    registry_->register_publisher(
      base_topic_, std::type_index(typeid(AdapterT)), adapted_topic_,
      token_);
  }

  ~Publisher()
  {
    shutdown();
  }

  /// Publish to local adapted subscribers, public transport subscribers, or both.
  /**
   * This overload copies the custom message if local adapted delivery is needed.
   * Use publish(std::unique_ptr<CustomMessage>) to preserve ownership into the
   * intra-process path.
   */
  void publish(const CustomMessage & message) const
  {
    if (async_publish_) {
      enqueue_publish(std::make_unique<CustomMessage>(message));
      return;
    }
    publish_now(message);
  }

  /// Queue or publish an owned custom message, moving it into local delivery last.
  void publish(std::unique_ptr<CustomMessage> message) const
  {
    if (async_publish_) {
      enqueue_publish(std::move(message));
      return;
    }
    publish_now(std::move(message));
  }

  /// Return true when publish() defers conversion and transport work to a timer callback.
  bool isAsyncPublishEnabled() const
  {
    return async_publish_;
  }

private:
  void enqueue_publish(std::unique_ptr<CustomMessage> message) const
  {
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_message_ = std::move(message);
    }
    if (publish_timer_) {
      publish_timer_->reset();
    }
  }

  /// Drain the latest queued message from the executor timer callback.
  void drain_pending_publish() const
  {
    if (publish_timer_) {
      publish_timer_->cancel();
    }

    std::unique_ptr<CustomMessage> message;
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      message = std::move(pending_message_);
    }
    if (message) {
      publish_now(std::move(message));
    }
  }

  /// Synchronous publish path for borrowed custom messages.
  void publish_now(const CustomMessage & message) const
  {
    const bool has_local = registry_ &&
      registry_->local_subscriber_count(base_topic_, std::type_index(typeid(AdapterT))) > 0;
    const bool has_public = public_publisher_.getNumSubscribers() > 0;

    if (has_public) {
      RosMessage ros_message;
      Adapter::convert_to_ros_message(message, ros_message);
      public_publisher_.publish(ros_message);
    }
    if (has_local && adapted_publisher_) {
      adapted_publisher_->publish(std::make_unique<CustomMessage>(message));
    }
  }

  /// Synchronous publish path for owned messages, preserving zero-copy local delivery.
  void publish_now(std::unique_ptr<CustomMessage> message) const
  {
    if (!message) {
      return;
    }

    const bool has_local = registry_ &&
      registry_->local_subscriber_count(base_topic_, std::type_index(typeid(AdapterT))) > 0;
    const bool has_public = public_publisher_.getNumSubscribers() > 0;

    if (has_public) {
      RosMessage ros_message;
      Adapter::convert_to_ros_message(*message, ros_message);
      public_publisher_.publish(ros_message);
    }
    if (has_local && adapted_publisher_) {
      adapted_publisher_->publish(std::move(message));
    }
  }

public:
  /// Return public transport subscribers plus same-process adapted subscribers.
  size_t getNumSubscribers() const
  {
    return getNumPublicSubscribers() + getNumLocalSubscribers();
  }

  /// Return only subscribers connected through image_transport topics.
  size_t getNumPublicSubscribers() const
  {
    return public_publisher_.getNumSubscribers();
  }

  /// Return only same-process subscribers registered for the adapted path.
  size_t getNumLocalSubscribers() const
  {
    return registry_ ?
           registry_->local_subscriber_count(base_topic_, std::type_index(typeid(AdapterT))) : 0;
  }

  std::string getTopic() const
  {
    return base_topic_;
  }

  std::string getAdaptedTopic() const
  {
    return adapted_topic_;
  }

  /// Unregister the adapted publisher and shut down public advertisements.
  void shutdown()
  {
    if (publish_timer_) {
      publish_timer_->cancel();
    }
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_message_.reset();
    }
    if (registry_ && token_ != 0) {
      registry_->unregister_publisher(token_);
      token_ = 0;
    }
    if (public_publisher_) {
      public_publisher_.shutdown();
    }
    adapted_publisher_.reset();
  }

  explicit operator bool() const
  {
    return static_cast<bool>(adapted_publisher_);
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::string base_topic_;
  bool async_publish_ = true;
  std::shared_ptr<detail::Registry> registry_;
  uint64_t token_ = 0;
  std::string adapted_topic_;
  typename rclcpp::Publisher<AdapterT>::SharedPtr adapted_publisher_;
  image_transport::Publisher public_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  mutable std::mutex pending_mutex_;
  mutable std::unique_ptr<CustomMessage> pending_message_;
};

template<typename AdapterT>
/// Type-adapted image subscriber with public fallback while no local publisher exists.
class Subscriber
{
  static_assert(
    detail::is_valid_adapter_v<AdapterT>,
    "adapted_image_transport::Subscriber requires an rclcpp TypeAdapter whose ros_message_type "
    "is sensor_msgs::msg::Image");

public:
  using Adapter = rclcpp::TypeAdapter<AdapterT>;
  using CustomMessage = typename Adapter::custom_type;
  using Callback = std::function<void (const std::shared_ptr<const CustomMessage> &)>;
  using UniqueCallback = std::function<void (std::unique_ptr<CustomMessage>)>;
  struct UniqueCallbackTag {};

  Subscriber() = default;

  /// Create a subscriber that starts on the public path until a local publisher appears.
  /**
   * The subscriber registers interest in the context-local registry. When a matching
   * publisher appears it switches to the private adapted topic; when it disappears it
   * falls back to the public image_transport path.
   */
  Subscriber(
    rclcpp::Node::SharedPtr node,
    std::string base_topic,
    Callback callback,
    std::string transport = "raw",
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
    rclcpp::SubscriptionOptions public_options = rclcpp::SubscriptionOptions())
  : node_(std::move(node)),
    base_topic_(std::move(base_topic)),
    shared_callback_(std::move(callback)),
    transport_(std::move(transport)),
    custom_qos_(custom_qos),
    public_options_(std::move(public_options)),
    registry_(detail::registry_for(node_)),
    guard_condition_(std::make_shared<rclcpp::GuardCondition>(
        node_->get_node_base_interface()->get_context()))
  {
    guard_condition_->set_on_trigger_callback([this](size_t) {refresh_subscription();});
    interest_id_ = registry_->add_interest(
      base_topic_, std::type_index(typeid(AdapterT)), guard_condition_);
    refresh_subscription();
  }

  /// Create a subscriber that receives owned custom messages when the local path is active.
  Subscriber(
    rclcpp::Node::SharedPtr node,
    std::string base_topic,
    UniqueCallback callback,
    UniqueCallbackTag,
    std::string transport = "raw",
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
    rclcpp::SubscriptionOptions public_options = rclcpp::SubscriptionOptions())
  : node_(std::move(node)),
    base_topic_(std::move(base_topic)),
    unique_callback_(std::move(callback)),
    transport_(std::move(transport)),
    custom_qos_(custom_qos),
    public_options_(std::move(public_options)),
    registry_(detail::registry_for(node_)),
    guard_condition_(std::make_shared<rclcpp::GuardCondition>(
        node_->get_node_base_interface()->get_context()))
  {
    guard_condition_->set_on_trigger_callback([this](size_t) {refresh_subscription();});
    interest_id_ = registry_->add_interest(
      base_topic_, std::type_index(typeid(AdapterT)), guard_condition_);
    refresh_subscription();
  }

  ~Subscriber()
  {
    shutdown();
  }

  /// Unregister local interest and shut down any active subscription.
  void shutdown()
  {
    if (registry_ && interest_id_ != 0) {
      registry_->remove_interest(interest_id_);
      interest_id_ = 0;
    }
    local_subscription_.reset();
    if (public_subscriber_) {
      public_subscriber_.shutdown();
      public_subscriber_ = image_transport::Subscriber();
    }
    if (guard_condition_) {
      guard_condition_->set_on_trigger_callback(nullptr);
    }
  }

  std::string getTopic() const
  {
    return base_topic_;
  }

  std::string getTransport() const
  {
    return using_local_ ? "adapted" : transport_;
  }

  /// True when this subscriber is using the private adapted intra-process topic.
  bool usingLocalTransport() const
  {
    return using_local_;
  }

  explicit operator bool() const
  {
    return static_cast<bool>(shared_callback_) || static_cast<bool>(unique_callback_);
  }

private:
  void refresh_subscription()
  {
    if (!registry_ || (!shared_callback_ && !unique_callback_)) {
      return;
    }

    const auto publishers = registry_->find_publishers(
      base_topic_,
      std::type_index(typeid(AdapterT)));
    if (!publishers.empty()) {
      const auto adapted_topic = publishers.front().adapted_topic;
      if (using_local_ && adapted_topic == current_adapted_topic_) {
        return;
      }
      create_local_subscription(adapted_topic);
      if (public_subscriber_) {
        public_subscriber_.shutdown();
        public_subscriber_ = image_transport::Subscriber();
      }
      using_local_ = true;
      current_adapted_topic_ = adapted_topic;
      return;
    }

    local_subscription_.reset();
    current_adapted_topic_.clear();
    if (!public_subscriber_) {
      create_public_subscription();
    }
    using_local_ = false;
  }

  /// Subscribe to the private adapted topic with intra-process communication enabled.
  void create_local_subscription(const std::string & adapted_topic)
  {
    auto options = public_options_;
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    if (unique_callback_) {
      local_subscription_ = node_->create_subscription<AdapterT>(
        adapted_topic,
        detail::qos_from_rmw(custom_qos_),
        [callback = unique_callback_](std::unique_ptr<CustomMessage> message) {
          callback(std::move(message));
        },
        options);
    } else {
      local_subscription_ = node_->create_subscription<AdapterT>(
        adapted_topic,
        detail::qos_from_rmw(custom_qos_),
        [callback = shared_callback_](const std::shared_ptr<const CustomMessage> message) {
          callback(message);
        },
        options);
    }
  }

  /// Subscribe to the public image_transport path and convert ROS messages to custom type.
  void create_public_subscription()
  {
    public_subscriber_ = image_transport::create_subscription(
      node_.get(),
      base_topic_,
      [shared_callback = shared_callback_,
      unique_callback = unique_callback_](const sensor_msgs::msg::Image::ConstSharedPtr & message) {
        if (unique_callback) {
          auto custom = std::make_unique<CustomMessage>();
          Adapter::convert_to_custom(*message, *custom);
          unique_callback(std::move(custom));
        } else {
          auto custom = std::make_shared<CustomMessage>();
          Adapter::convert_to_custom(*message, *custom);
          shared_callback(custom);
        }
      },
      transport_,
      custom_qos_,
      public_options_);
  }

  rclcpp::Node::SharedPtr node_;
  std::string base_topic_;
  Callback shared_callback_;
  UniqueCallback unique_callback_;
  std::string transport_ = "raw";
  rmw_qos_profile_t custom_qos_ = rmw_qos_profile_default;
  rclcpp::SubscriptionOptions public_options_;
  std::shared_ptr<detail::Registry> registry_;
  rclcpp::GuardCondition::SharedPtr guard_condition_;
  uint64_t interest_id_ = 0;
  bool using_local_ = false;
  std::string current_adapted_topic_;
  typename rclcpp::Subscription<AdapterT>::SharedPtr local_subscription_;
  image_transport::Subscriber public_subscriber_;
};

template<typename AdapterT>
/// Create a type-adapted image publisher.
Publisher<AdapterT> create_publisher(
  rclcpp::Node::SharedPtr node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
  rclcpp::PublisherOptions options = rclcpp::PublisherOptions(),
  bool async_publish = true)
{
  return Publisher<AdapterT>(
    std::move(node), base_topic, custom_qos, std::move(options), async_publish);
}

template<typename AdapterT>
/// Create a type-adapted image subscriber.
Subscriber<AdapterT> create_subscription(
  rclcpp::Node::SharedPtr node,
  const std::string & base_topic,
  typename Subscriber<AdapterT>::Callback callback,
  const std::string & transport = "raw",
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
  rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions())
{
  return Subscriber<AdapterT>(
    std::move(node), base_topic, std::move(callback), transport, custom_qos, std::move(options));
}

template<typename AdapterT>
/// Create a type-adapted image subscriber with owned-message callbacks.
Subscriber<AdapterT> create_unique_subscription(
  rclcpp::Node::SharedPtr node,
  const std::string & base_topic,
  typename Subscriber<AdapterT>::UniqueCallback callback,
  const std::string & transport = "raw",
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
  rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions())
{
  return Subscriber<AdapterT>(
    std::move(node), base_topic, std::move(callback),
    typename Subscriber<AdapterT>::UniqueCallbackTag{}, transport, custom_qos, std::move(options));
}

}  // namespace adapted_image_transport

#endif  // ADAPTED_IMAGE_TRANSPORT__ADAPTED_IMAGE_TRANSPORT_HPP_
