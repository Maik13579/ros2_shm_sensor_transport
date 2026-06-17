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

#include "shm_sensor_transport/shm_name.hpp"

#include <cctype>
#include <functional>
#include <sstream>

namespace shm_sensor_transport
{

std::string normalize_shared_memory_name(const std::string & name)
{
  std::string normalized;
  normalized.reserve(name.size() + 1U);
  if (name.empty() || name.front() != '/') {
    normalized.push_back('/');
  }
  for (std::size_t index = 0; index < name.size(); ++index) {
    const char ch = name[index];
    if (index == 0U && ch == '/') {
      normalized.push_back('/');
      continue;
    }
    if (std::isalnum(static_cast<unsigned char>(ch))) {
      normalized.push_back(ch);
    } else {
      normalized.push_back('_');
    }
  }
  while (normalized.size() > 1U && normalized.back() == '_') {
    normalized.pop_back();
  }
  return normalized;
}

std::string make_shared_memory_name(const std::string & topic_name)
{
  std::string stem;
  stem.reserve(topic_name.size());
  for (const char ch : topic_name) {
    if (std::isalnum(static_cast<unsigned char>(ch))) {
      stem.push_back(ch);
    } else {
      stem.push_back('_');
    }
  }
  while (!stem.empty() && stem.front() == '_') {
    stem.erase(stem.begin());
  }
  if (stem.empty()) {
    stem = "topic";
  }

  std::ostringstream suffix;
  suffix << std::hex << std::hash<std::string>{}(topic_name);
  return normalize_shared_memory_name("/ros2_shm_" + stem + "_" + suffix.str());
}

std::string shared_memory_path(const std::string & shm_name)
{
  const auto normalized = normalize_shared_memory_name(shm_name);
  return "/dev/shm/" + normalized.substr(1U);
}

}  // namespace shm_sensor_transport
