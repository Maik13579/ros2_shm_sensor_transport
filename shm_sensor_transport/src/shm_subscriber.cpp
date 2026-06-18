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

#include "shm_sensor_transport/shm_subscriber.hpp"

#include <stdexcept>

namespace shm_sensor_transport
{

std::string resolve_metadata_topic(const std::string & topic)
{
  const auto end = topic.find_last_not_of('/');
  if (end == std::string::npos) {
    throw std::invalid_argument("topic must not be empty");
  }

  const auto normalized = topic.substr(0, end + 1U);
  if (normalized.size() >= 5U && normalized.compare(normalized.size() - 5U, 5U, "/_shm") == 0) {
    return normalized;
  }
  return normalized + "/_shm";
}

}  // namespace shm_sensor_transport
