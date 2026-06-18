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

#pragma once

#include <string>

#include <rclcpp/node.hpp>

namespace shm_sensor_transport
{

/// @brief Parameter set shared by image and point-cloud relay components.
struct RelayParameters
{
  std::string input_topic;
  std::string meta_topic;
  std::string status_topic;
  std::string shm_name;
  int slot_count{8};
  std::uint64_t slot_size_bytes{0};
  bool allow_resize{false};
  bool publish_status{false};
  double status_rate{1.0};
  double rate_limit_hz{0.0};
  bool use_sensor_data_qos{true};
  bool reliable{false};
  int depth{1};
  bool warn_on_oversized_frame{true};
  bool warn_on_late_initialization{true};
};

/// @brief Declare and read the grouped relay parameters on a ROS 2 node.
RelayParameters declare_relay_parameters(
  rclcpp::Node & node,
  const std::string & default_input_topic);

}  // namespace shm_sensor_transport
