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

#include "shm_sensor_transport/parameter_utils.hpp"

#include <algorithm>

namespace shm_sensor_transport
{

namespace
{

std::string make_hidden_metadata_topic(const std::string & input_topic)
{
  if (!input_topic.empty() && input_topic.back() == '/') {
    return input_topic + "_shm";
  }
  return input_topic + "/_shm";
}

}  // namespace

RelayParameters declare_relay_parameters(
  rclcpp::Node & node,
  const std::string & default_input_topic)
{
  RelayParameters params;
  params.input_topic = node.declare_parameter<std::string>(
    "common.input_topic", default_input_topic);
  params.meta_topic = make_hidden_metadata_topic(params.input_topic);
  params.status_topic = node.declare_parameter<std::string>("common.status_topic", "");
  params.shm_name = node.declare_parameter<std::string>("common.shm_name", "");
  params.slot_count = node.declare_parameter<int>("common.slot_count", 8);
  params.slot_size_bytes = static_cast<std::uint64_t>(
    node.declare_parameter<int>("common.slot_size_bytes", 0));
  params.allow_resize = node.declare_parameter<bool>("common.allow_resize", false);
  params.publish_status = node.declare_parameter<bool>("common.publish_status", false);
  params.status_rate = node.declare_parameter<double>("common.status_rate", 1.0);

  params.use_sensor_data_qos = node.declare_parameter<bool>("qos.use_sensor_data_qos", true);
  params.reliable = node.declare_parameter<bool>("qos.reliable", false);
  params.depth = node.declare_parameter<int>("qos.depth", 1);

  params.warn_on_oversized_frame =
    node.declare_parameter<bool>("diagnostics.warn_on_oversized_frame", true);
  params.warn_on_late_initialization =
    node.declare_parameter<bool>("diagnostics.warn_on_late_initialization", true);

  params.slot_count = std::max(1, params.slot_count);
  params.depth = std::max(1, params.depth);
  params.status_rate = std::max(0.0, params.status_rate);
  return params;
}

}  // namespace shm_sensor_transport
