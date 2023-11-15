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

#include <map>
#include <string>
#include <vector>
#include <utility>
#include <memory>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include "modules/perception/lidar_cluster/cluster/proto/euclidean_cluster_config.pb.h"

#include "cyber/common/file.h"
#include "modules/perception/common/lidar/common/cloud_mask.h"
#include "modules/perception/common/util.h"
#include "modules/perception/lidar_cluster/interface/base_object_cluster.h"

namespace apollo {
namespace perception {
namespace lidar {

class EuclideanCluster : public BaseObjectCluster {
 public:
  EuclideanCluster() = default;
  ~EuclideanCluster() = default;

 public:
  bool Init(const ObjectClusterInitOptions& options =
                ObjectClusterInitOptions()) override;
  bool Cluster(const ObjectClusterOptions& options, LidarFrame* frame) override;
  std::string Name() const override { return "EuclideanCluster"; }

 private:
  EuclideanClusterConfig euclidean_cluster_param_;
  float valid_distance_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
