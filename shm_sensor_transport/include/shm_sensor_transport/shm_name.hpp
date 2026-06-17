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

namespace shm_sensor_transport
{

/// @brief Generate a stable POSIX shared-memory name from a ROS topic name.
std::string make_shared_memory_name(const std::string & topic_name);

/// @brief Normalize a user-provided name into POSIX shared-memory name form.
std::string normalize_shared_memory_name(const std::string & name);

/// @brief Convert a POSIX shared-memory name into its Linux /dev/shm path.
std::string shared_memory_path(const std::string & shm_name);

}  // namespace shm_sensor_transport
