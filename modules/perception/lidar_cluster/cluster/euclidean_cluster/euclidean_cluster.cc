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

#include "modules/perception/lidar_cluster/cluster/euclidean_cluster/euclidean_cluster.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::cyber::common::GetProtoFromFile;

bool EuclideanCluster::Init(const ObjectClusterInitOptions& options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  // get euclidean cluster params
  euclidean_cluster_param_.Clear();
  ACHECK(GetProtoFromFile(config_file, &euclidean_cluster_param_))
      << "Failed to parse EuclideanClusterConfig config file.";
  valid_distance_ = euclidean_cluster_param_.valid_distance();

  return true;
}

bool EuclideanCluster::Cluster(const ObjectClusterOptions& options,
                               LidarFrame* frame) {
  // check input
  if (frame == nullptr) {
    AERROR << "Input null frame ptr.";
    return false;
  }
  if (frame->cloud == nullptr) {
    AERROR << "Input null frame cloud.";
    return false;
  }
  if (frame->cloud->size() == 0) {
    AERROR << "Input none points.";
    return false;
  }
  // obtain intersection of roi and non-ground indices
  CloudMask mask;
  mask.Set(frame->cloud->size(), 0);
  mask.AddIndicesOfIndices(frame->roi_indices, frame->non_ground_indices, 1);
  base::PointIndices indices;
  mask.GetValidIndices(&indices);
  // get pointcloud of indices
  std::map<int, int>
      index_mapper_;  // key: index-in-pcl_cloud, value: index-in-cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl_cloud->reserve(indices.indices.size());
  int valid_count = 0;
  for (size_t i = 0; i < indices.indices.size(); ++i) {
    int idx = indices.indices.at(i);
    auto pt = frame->cloud->at(idx);
    if (pt.y > -valid_distance_ * 0.5 &&
        pt.y < valid_distance_ * 0.5 &&
        pt.x > -10.0 &&
        pt.x < valid_distance_ - 10.0) {
      index_mapper_[valid_count] = idx;
      valid_count++;
      pcl::PointXYZ pcl_pt;
      pcl_pt.x = pt.x;
      pcl_pt.y = pt.y;
      pcl_pt.z = pt.z;
      pcl_cloud->push_back(pcl_pt);
    }
  }

  // kd tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pcl_cloud);
  // do euclidean cluster
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(euclidean_cluster_param_.cluster_distance());
  ec.setMinClusterSize(euclidean_cluster_param_.min_cluster_size());
  ec.setMaxClusterSize(euclidean_cluster_param_.max_cluster_size());
  ec.setSearchMethod(tree);
  ec.setInputCloud(pcl_cloud);
  ec.extract(cluster_indices);

  // get object from cluster_indices
  for (pcl::PointIndices cluster : cluster_indices) {
    if (cluster.indices.size() < euclidean_cluster_param_.min_point_number()) {
      continue;
    }
    // get cluster point index in cloud
    std::vector<int> point_idx;
    for (auto idx : cluster.indices) {
      point_idx.push_back(index_mapper_.at(idx));
    }
    base::Object object;
    object.confidence = 1.0;
    object.lidar_supplement.is_in_roi = true;
    object.lidar_supplement.num_points_in_roi = cluster.indices.size();
    object.lidar_supplement.cloud.CopyPointCloud(*frame->cloud, point_idx);
    object.lidar_supplement.cloud_world.CopyPointCloud(*frame->world_cloud,
                                                       point_idx);
    // classification
    object.type = base::ObjectType::UNKNOWN;
    object.lidar_supplement.raw_probs.push_back(std::vector<float>(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
    object.lidar_supplement.raw_probs.back()[static_cast<int>(object.type)] =
        1.0f;
    object.lidar_supplement.raw_classification_methods.push_back(Name());
    // copy to type
    object.type_probs.assign(object.lidar_supplement.raw_probs.back().begin(),
                             object.lidar_supplement.raw_probs.back().end());
    // copy to background objects
    std::shared_ptr<base::Object> obj(new base::Object);
    *obj = object;
    frame->segmented_objects.push_back(std::move(obj));
  }
  return true;
}

PERCEPTION_REGISTER_LIDARCLUSTER(EuclideanCluster);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
