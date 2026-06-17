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

#include "shm_sensor_transport/qos_utils.hpp"

#include <algorithm>

#include <rclcpp/qos.hpp>

namespace shm_sensor_transport
{

rclcpp::QoS make_metadata_qos(const bool use_sensor_data_qos, const bool reliable, const int depth)
{
  const auto queue_depth = static_cast<std::size_t>(std::max(1, depth));
  rclcpp::QoS qos = use_sensor_data_qos ? rclcpp::SensorDataQoS() : rclcpp::QoS(queue_depth);
  qos.keep_last(queue_depth);
  qos.durability_volatile();
  if (reliable) {
    qos.reliable();
  } else {
    qos.best_effort();
  }
  return qos;
}

}  // namespace shm_sensor_transport
