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

#include <memory>

#include <rviz_default_plugins/displays/pointcloud/point_cloud2_display.hpp>

#include "shm_sensor_transport/shm_subscriber.hpp"

namespace shm_sensor_transport_rviz
{

class ShmPointCloud2Display : public rviz_default_plugins::displays::PointCloud2Display
{
public:
  ShmPointCloud2Display();
  ~ShmPointCloud2Display() override;

protected:
  void subscribe() override;
  void unsubscribe() override;

private:
  std::shared_ptr<shm_sensor_transport::ShmPointCloud2Subscriber> shm_subscription_;
};

}  // namespace shm_sensor_transport_rviz
