#include "modules/perception/lidar/lib/pointcloud_cluster/graph_segmentation_cluster.h"
#include "cyber/common/file.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/common/point_cloud_processing/common.h"

namespace century {
namespace perception {
namespace lidar {

using century::cyber::common::GetAbsolutePath;

static constexpr size_t LATERAL_SEARCH_RANGE = 4;
static constexpr size_t LONGITUDINAL_SEARCH_RANGE = 2;
static constexpr size_t TOTAL_SEARCH_RANGE = LATERAL_SEARCH_RANGE + LONGITUDINAL_SEARCH_RANGE;

inline float CalRange(const base::PointF& point) {
  return sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

bool GraphSegmentationCluster::Init(const ClusterInitOptions& options) {
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(
      config_manager->GetModelConfig("PointCloudPreprocessor", &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;

  config_file = GetAbsolutePath(work_root, "conf/perception/lidar");
  config_file =
      GetAbsolutePath(config_file, "pointcloud_cluster_config.pb.txt");
  ACHECK(
      century::cyber::common::GetProtoFromFile(config_file, &cluster_config_))
      << ", work_root: " << work_root << ", config_file: " << config_file;

  row_num_ = cluster_config_.graph_seg_row_num();
  col_num_ = cluster_config_.graph_seg_col_num();
  f_up_ = cluster_config_.graph_seg_f_up();
  f_down_ = cluster_config_.graph_seg_f_down();
  phi_v_ = cluster_config_.graph_seg_phi_v();
  phi_h_ = cluster_config_.graph_seg_phi_h();
  sigma_v_ = cluster_config_.graph_seg_sigma_v();
  sigma_h_ = cluster_config_.graph_seg_sigma_h();
  horizontal_search_angle_ = cluster_config_.graph_seg_horizontal_search_angle();
  vertical_search_angle_ = cluster_config_.graph_seg_vertical_search_angle();
  min_points_size_ = cluster_config_.graph_seg_min_points_size();

  use_map_manager_ = cluster_config_.use_map_manager();
  use_map_manager_ = use_map_manager_ && options.enable_hdmap_input;

  if (use_map_manager_) {
    lidar::MapManagerInitOptions map_manager_init_options;
    if (!map_manager_.Init(map_manager_init_options)) {
      AINFO << "Failed to init map manager.";
      use_map_manager_ = false;
    }
  }

  roi_cloud_ = base::PointFCloudPool::Instance().Get();
  roi_filter_ =
      lidar::BaseROIFilterRegisterer::GetInstanceByName("HdmapROIFilter");
  CHECK_NOTNULL(roi_filter_);
  lidar::ROIFilterInitOptions roi_filter_init_options;
  ACHECK(roi_filter_->Init(roi_filter_init_options))
      << "Failed to init roi filter.";
  return true;
}

bool GraphSegmentationCluster::Cluster(const ClusterOptions& options,
                                       LidarFrame* frame) {

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

  if (use_map_manager_) {
    MapManagerOptions map_manager_options;
    if (!map_manager_.Update(map_manager_options, frame)) {
      AERROR << "Failed to update map structure.";
      return false;
    }

    common::TransformPointCloud(*frame->cloud,
      frame->lidar2world_pose,
      frame->world_cloud.get());

    lidar::ROIFilterOptions roi_filter_options;
    if (frame->hdmap_struct != nullptr &&
        roi_filter_->Filter(roi_filter_options,
                            frame)) {
      roi_cloud_->CopyPointCloud(*frame->cloud,
                                 frame->roi_indices);
      frame->cloud = roi_cloud_;
    }
  }

  auto points = frame->cloud;
  Range r;
  r.col = -1;
  r.row = -1;
  r.index = -1;
  r.range = -1;
  r.z = -1;
  std::vector<Range> hash_range(col_num_ * row_num_, r);
  ToRangeImage(points, hash_range);
  std::vector<int> cluster_indices = RangeSeg(points, hash_range);
  std::vector<std::vector<int32_t> > cluster_id;
  MostFrequentValue(cluster_indices, cluster_id);
  for(auto& cluster_indice : cluster_id) {
    base::Object object;
    object.confidence = 1.0;
    object.lidar_supplement.is_in_roi = true;
    object.lidar_supplement.num_points_in_roi = cluster_indice.size();
    object.lidar_supplement.cloud.CopyPointCloud(*frame->cloud, cluster_indice);
    // object.lidar_supplement.cloud_world.CopyPointCloud(*frame->world_cloud,
    //                                                    point_idx);

    // classification
    object.type = base::ObjectType::UNKNOWN;
    object.lidar_supplement.raw_probs.emplace_back(std::vector<float>(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
    object.lidar_supplement.raw_probs.back()[static_cast<int>(object.type)] =
        1.0f;
    object.lidar_supplement.raw_classification_methods.emplace_back(Name());
    // copy to type
    object.type_probs.assign(object.lidar_supplement.raw_probs.back().begin(),
                             object.lidar_supplement.raw_probs.back().end());
    // copy to background objects
    std::shared_ptr<base::Object> obj(new base::Object);
    *obj = object;
    frame->segmented_objects.emplace_back(std::move(obj));
  }
  return true;
}

void GraphSegmentationCluster::ToRangeImage(std::shared_ptr<base::AttributePointCloud<base::PointF>> points,
                                          std::vector<Range>& map) const{
  for (size_t i = 0; i < points->size(); i++) {
    const base::PointF point = points->at(i);
    float range = CalRange(point) + 0.001;
    int col = (1 - atan2(point.x, point.y) / M_PI) * (float)col_num_;
    col = col * 0.5; // vehicle point is on col num / 2
    if (col >= col_num_) {
      col = col_num_ - 1;
    }
    int row = (1 - (asin(point.z / range) / M_PI * 180 - f_down_) / (f_up_ - f_down_)) *
        (float)row_num_;
    if (row >= row_num_) { row = row_num_ - 1; }
    if (row < 0) {row = 0;}
    int point_idx = row * col_num_ + col;
    if (-1 == map[point_idx].index) {
      Range& r = map[point_idx];
      r.col = col;
      r.row = row;
      r.index = i;
      r.range = range;
      r.z = point.z;
    }
  }
}

void GraphSegmentationCluster::MergeClusters(std::vector<int>* cluster_indices, int idx1,
    int idx2) const {
  for (size_t i = 0; i < cluster_indices->size(); i++) {
    if (cluster_indices->at(i) == idx1) {
      cluster_indices->at(i) = idx2;
    }
  }
}

bool GraphSegmentationCluster::MostFrequentValue(const std::vector<int>& values,
    std::vector<std::vector<int32_t> >& cluster_indices) const {

  std::unordered_map<int, std::vector<int32_t> > hist;
  for (size_t i = 0; i < values.size(); ++i) {
    if (-1 != values[i]) {
      hist[values[i]].emplace_back(i);
    }
  }

  for (auto iter = hist.begin(); iter != hist.end(); ++iter) {
    if (iter->second.size() > min_points_size_) {
      cluster_indices.emplace_back(iter->second);
    }
  }
  return true;
}

std::vector<int> GraphSegmentationCluster::RangeSeg(std::shared_ptr<base::AttributePointCloud<base::PointF>> points,
    std::vector<Range>& map) const {
  int direction[TOTAL_SEARCH_RANGE][2] = {{0,1}, {0,2}, {0,-1}, {0, -2}, {-1, 0}, {1, 0}};
  int size = points->size();
  std::vector<int> cluster_indices(size, -1);
  int current_cluster = 0;
  for (int i = 0; i < col_num_ * row_num_; ++i) {
    if (-1 == map[i].index) { continue; }
    int col = map[i].col;
    int row = map[i].row;
    int index = map[i].index;
    if (cluster_indices[index] != -1) { continue; }

    float dorigin = map[i].range;
    for (int k = 0; k < LATERAL_SEARCH_RANGE; ++k) {
      int neighbor = (row + direction[k][0]) * col_num_ + (col + direction[k][1]);
      if (neighbor > col_num_ * row_num_ - 1 || neighbor < 0) { continue; }
      if (-1 == map[neighbor].index) { continue; }
      float dnei = map[neighbor].range;
      float dmax = (dorigin) * sin(phi_h_ * M_PI / 180) /
          sin(horizontal_search_angle_ * M_PI / 180 - phi_h_ * M_PI / 180) + 3 * sigma_h_;
      int neighbor_idx = map[neighbor].index;
      if (fabs(dorigin - dnei) < dmax) {
        int oc = cluster_indices[index];
        int nc = cluster_indices[neighbor_idx];
        if (oc != -1 && nc != -1) {
          if (oc != nc) {
            MergeClusters(&cluster_indices, oc, nc);
          }
        } else {
          if (nc != -1) {
            cluster_indices[index] = nc;
          } else if (oc != -1) {
            cluster_indices[neighbor_idx] = oc;
          }
        }
      }
    }

    for (int k = LATERAL_SEARCH_RANGE; k < TOTAL_SEARCH_RANGE; ++k) {
      int neighbor = (row + direction[k][0]) * col_num_ + (col + direction[k][1]);
      if (neighbor > col_num_ * row_num_ - 1 || neighbor < 0) { continue; }
      if (-1 == map[neighbor].index) { continue; };

      float dnei = map[neighbor].range;
      float dmax = (dorigin) * sin(phi_v_ * M_PI / 180) /
          sin(vertical_search_angle_ * M_PI / 180 - phi_v_ * M_PI / 180) + 3 * sigma_v_;
      int neighbor_idx = map[neighbor].index;
      if (fabs(dnei - dorigin) < dmax) {
        int oc = cluster_indices[index]; // original point's cluster
        int nc = cluster_indices[neighbor_idx]; // neighbor point's cluster
        if (oc != -1 && nc != -1) {
          if (oc != nc) {
            MergeClusters(&cluster_indices, oc, nc);
          }
        } else {
          if (nc != -1) {
            cluster_indices[index] = nc;
          } else {
            if (oc != -1) {
              cluster_indices[neighbor_idx] = oc;
            }
          }
        }
      }
    }

    if (-1 == cluster_indices[index]) {
      current_cluster ++;
      cluster_indices[index] = current_cluster;
      for (int k = 0; k < LONGITUDINAL_SEARCH_RANGE; ++k) {
        int neighbor = (row + direction[k][0]) * col_num_ + (col + direction[k][1]);
        if (neighbor > col_num_ * row_num_ - 1 || neighbor < 0) { continue; }
        if (-1 == map[neighbor].index) { continue; }
        float dnei = map[neighbor].range;
        float dmax = (map[i].range) * sin(phi_h_ * M_PI / 180) /
            sin(horizontal_search_angle_ * M_PI/180 - phi_h_ * M_PI / 180) + 3 * sigma_h_;
        int neighbor_idx = map[neighbor].index;
        if (fabs(dnei - dorigin) < dmax) {
          cluster_indices[neighbor_idx] = current_cluster;
        }
      }

      for (int k = LATERAL_SEARCH_RANGE; k < TOTAL_SEARCH_RANGE; ++k) {
        int neighbor = (row + direction[k][0]) * col_num_ + (col + direction[k][1]);
        if (neighbor > col_num_ * row_num_ - 1 || neighbor < 0) { continue; }
        if (-1 == map[neighbor].index) { continue; }
        float dnei = map[neighbor].range;
        float dmax = (dorigin) * sin(phi_v_ * M_PI / 180) /
            sin(vertical_search_angle_ * M_PI / 180 - phi_v_ * M_PI / 180) + 3 * sigma_v_; // 0.5
        int neighbor_idx = map[neighbor].index;
        if (fabs(dnei - dorigin) < dmax) {
          cluster_indices[neighbor_idx] = current_cluster;
        }
      }
    }
  }
  return cluster_indices;

}

PERCEPTION_REGISTER_POINTCLOUDCLUSTERER(GraphSegmentationCluster);

}  // namespace lidar
}  // namespace perception
}  // namespace century