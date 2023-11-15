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

#include "modules/perception/lidar_cluster/lidar_cluster_component.h"

#include "cyber/profiler/profiler.h"

namespace apollo {
namespace perception {
namespace lidar {

bool LidarClusterComponent::Init() {
  ACHECK(ComponentBase::GetProtoConfig(&config_))
      << "failed to load lidar_cluster_component config file "
      << ComponentBase::ConfigFilePath();
  AINFO << "Load config succedded.\n" << config_.DebugString();
  output_channel_name_ = config_.output_channel_name();
  writer_ = node_->CreateWriter<LidarFrameMessage>(output_channel_name_);

  // cluster
  auto plugin_param = config_.plugin_param();
  std::string cluster_name = plugin_param.name();
  object_cluster_ =
      BaseObjectClusterRegisterer::GetInstanceByName(cluster_name);
  CHECK_NOTNULL(object_cluster_);

  // init cluster
  ObjectClusterInitOptions cluster_init_options;
  cluster_init_options.config_path = plugin_param.config_path();
  cluster_init_options.config_file = plugin_param.config_file();
  ACHECK(object_cluster_->Init(cluster_init_options))
      << "Failed to init object cluster.";

  // object builder init
  ObjectBuilderInitOptions builder_init_options;
  ACHECK(builder_.Init(builder_init_options));

  AINFO << "Init LidarClusterComponent succedded.";
  return true;
}

bool LidarClusterComponent::Proc(
    const std::shared_ptr<LidarFrameMessage>& in_message) {
  PERF_FUNCION()
  if (in_message == nullptr) {
    AERROR << "LidarClusterComponent, in_message is null.";
    return false;
  }
  // internal proc
  bool status = InternalProc(in_message);

  if (status) {
    writer_->Write(in_message);
    AINFO << "Send object cluster message.";
  }
  return status;
}

bool LidarClusterComponent::InternalProc(
    const std::shared_ptr<LidarFrameMessage>& message) {
  PERF_BLOCK("object_cluster")
  ObjectClusterOptions cluster_options;
  if (!object_cluster_->Cluster(cluster_options, message->lidar_frame_.get())) {
    AERROR << "Object cluster error!";
    return false;
  }
  PERF_BLOCK_END

  // object builder
  PERF_BLOCK("object_builder")
  ObjectBuilderOptions builder_options;
  if (!builder_.Build(builder_options, message->lidar_frame_.get())) {
    AERROR << "Lidar cluster, object builder error.";
    return false;
  }
  PERF_BLOCK_END

  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
