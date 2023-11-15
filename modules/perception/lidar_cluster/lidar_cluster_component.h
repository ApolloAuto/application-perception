/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "modules/perception/lidar_cluster/proto/lidar_cluster_component.pb.h"

#include "cyber/component/component.h"
#include "cyber/cyber.h"

#include "modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h"
#include "modules/perception/lidar_cluster/interface/base_object_cluster.h"
#include "modules/perception/lidar_detection/object_builder/object_builder.h"

namespace apollo {
namespace perception {
namespace lidar {

using onboard::LidarFrameMessage;

class LidarClusterComponent final : public cyber::Component<LidarFrameMessage> {
 public:
  LidarClusterComponent() = default;
  virtual ~LidarClusterComponent() = default;

 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<LidarFrameMessage>& in_message) override;

 private:
  bool InternalProc(const std::shared_ptr<LidarFrameMessage>& message);

 private:
  apollo::perception::lidar::LidarClusterComponentConfig config_;
  std::shared_ptr<apollo::cyber::Writer<onboard::LidarFrameMessage>> writer_;
  std::string output_channel_name_;
  BaseObjectCluster* object_cluster_;
  ObjectBuilder builder_;
};

CYBER_REGISTER_COMPONENT(LidarClusterComponent)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
