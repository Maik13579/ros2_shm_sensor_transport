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

#include <gtest/gtest.h>

#include "shm_sensor_transport/shm_name.hpp"

TEST(ShmName, NormalizesNames)
{
  EXPECT_EQ(
    shm_sensor_transport::normalize_shared_memory_name("camera/image raw"),
    "/camera_image_raw");
  EXPECT_EQ(shm_sensor_transport::normalize_shared_memory_name("/already_valid"), "/already_valid");
}

TEST(ShmName, GeneratesStableTopicName)
{
  const auto first = shm_sensor_transport::make_shared_memory_name("/camera/image_raw");
  const auto second = shm_sensor_transport::make_shared_memory_name("/camera/image_raw");
  EXPECT_EQ(first, second);
  EXPECT_EQ(first.rfind("/ros2_shm_camera_image_raw_", 0), 0U);
}

TEST(ShmName, MapsToDevShmPath)
{
  EXPECT_EQ(shm_sensor_transport::shared_memory_path("/ros2_shm_test"), "/dev/shm/ros2_shm_test");
}
