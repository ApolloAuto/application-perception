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

#include <string>

#include "cyber/common/macros.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/common/lidar/common/lidar_frame.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::perception::BaseInitOptions;

struct ObjectClusterInitOptions : public BaseInitOptions {
  std::string sensor_name = "velodyne64";
};

struct ObjectClusterOptions {};

class BaseObjectCluster {
 public:
  BaseObjectCluster() = default;
  virtual ~BaseObjectCluster() = default;

  /**
   * @brief Init of the Base Object Cluster object
   *
   * @param options object cluster options
   * @return true
   * @return false
   */
  virtual bool Init(
      const ObjectClusterInitOptions& options = ObjectClusterInitOptions()) = 0;

  /**
   * @brief Cluster objects
   *
   * @param options object cluster options
   * @param frame lidar frame
   * @return true
   * @return false
   */
  virtual bool Cluster(const ObjectClusterOptions& options,
                       LidarFrame* frame) = 0;

  /**
   * @brief Name of Object Cluster
   *
   * @return std::string name
   */
  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseObjectCluster);
};  // class BaseObjectCluster

PERCEPTION_REGISTER_REGISTERER(BaseObjectCluster);
#define PERCEPTION_REGISTER_LIDARCLUSTER(name) \
  PERCEPTION_REGISTER_CLASS(BaseObjectCluster, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
