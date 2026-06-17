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

#include <rclcpp/qos.hpp>

namespace shm_sensor_transport
{

/// @brief Build the metadata QoS profile used by relay publishers and subscribers.
rclcpp::QoS make_metadata_qos(bool use_sensor_data_qos, bool reliable, int depth);

}  // namespace shm_sensor_transport
