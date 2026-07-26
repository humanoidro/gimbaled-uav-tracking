/**
 * @file IMMPDAFTracker.cpp
 * @brief 单目标 IMM-PDAF 跟踪实现。
 *        Single-target IMM-PDAF tracking.
 */

#include "IMMPDAFTracker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <utility>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace
{
constexpr double MINIMUM_VARIANCE = 1e-9;
constexpr double MINIMUM_DEPTH_M = 1e-6;
constexpr double LOG_TWO_PI = 1.8378770664093453;
constexpr double IDEAL_IMAGE_MARGIN_RATIO = 0.10;
constexpr double VISUAL_SCORE_BBOX_MARGIN_PX = 20.0;

void append_diagnostic(std::string& diagnostic, const std::string& message)
{
  if (message.empty())
  {
    return;
  }
  if (!diagnostic.empty())
  {
    diagnostic += "; ";
  }
  diagnostic += message;
}

bool finite_scalar(double value)
{
  return std::isfinite(value);
}
}  // namespace

IMMPDAFTracker::IMMPDAFTracker(const TrackingConfig& config, const CameraCalibration& camera)
{
  configure(config, camera);
}

void IMMPDAFTracker::configure(const TrackingConfig& config, const CameraCalibration& camera)
{
  config_ = config;
  camera_ = camera;
  configuration_diagnostic_.clear();
  configuration_valid_ = validate_configuration(configuration_diagnostic_);
  reset();
}

void IMMPDAFTracker::reset()
{
  initialized_ = false;
  lifecycle_ = TrackLifecycle::INACTIVE;
  hits_ = 0;
  misses_ = 0;
  lost_duration_s_ = 0.0;

  double initial_cv_probability = config_.initial_cv_probability;
  if (!finite_scalar(initial_cv_probability) || initial_cv_probability <= 0.0 || initial_cv_probability >= 1.0)
  {
    initial_cv_probability = 0.9;
  }
  mode_probabilities_ = Eigen::Vector2d(initial_cv_probability, 1.0 - initial_cv_probability);
  for (auto& mode : modes_)
  {
    mode.state.setZero();
    mode.covariance.setIdentity();
  }
}

bool IMMPDAFTracker::validate_configuration(std::string& diagnostic) const
{
  auto require = [&diagnostic](bool condition, const std::string& message)
  {
    if (!condition)
    {
      append_diagnostic(diagnostic, message);
    }
    return condition;
  };

  bool valid = true;
  valid &= require(finite_scalar(config_.minimum_range_m) && config_.minimum_range_m >= 0.0,
                   "minimum_range_m 必须为非负有限值");
  valid &= require(finite_scalar(config_.maximum_range_m) &&
                       config_.maximum_range_m > config_.minimum_range_m,
                   "maximum_range_m 必须大于 minimum_range_m");
  valid &= require(finite_scalar(config_.minimum_height_m) && config_.minimum_height_m >= 0.0,
                   "minimum_height_m 必须为非负有限值");
  valid &= require(finite_scalar(config_.cluster_tolerance_m) && config_.cluster_tolerance_m > 0.0,
                   "cluster_tolerance_m 必须大于 0");
  valid &= require(config_.cluster_min_points >= 1, "cluster_min_points 必须至少为 1");
  valid &= require(finite_scalar(config_.motion_expansion_sigma) && config_.motion_expansion_sigma >= 0.0,
                   "motion_expansion_sigma 必须为非负有限值");
  valid &= require(finite_scalar(config_.motion_gate_threshold) && config_.motion_gate_threshold > 0.0,
                   "motion_gate_threshold 必须大于 0");
  valid &= require(finite_scalar(config_.visual_baseline) && config_.visual_baseline >= 0.0,
                   "visual_baseline 必须为非负有限值");
  valid &= require(finite_scalar(config_.visual_sigma) && config_.visual_sigma > 0.0,
                   "visual_sigma 必须大于 0");
  valid &= require(finite_scalar(config_.visual_association_bbox_scale_x) &&
                       config_.visual_association_bbox_scale_x >= 1.0,
                   "visual_association_bbox_scale_x 必须至少为 1");
  valid &= require(finite_scalar(config_.visual_association_bbox_scale_y) &&
                       config_.visual_association_bbox_scale_y >= 1.0,
                   "visual_association_bbox_scale_y 必须至少为 1");
  valid &= require(finite_scalar(config_.minimum_detection_confidence) &&
                       config_.minimum_detection_confidence >= 0.0 &&
                       config_.minimum_detection_confidence <= 1.0,
                   "minimum_detection_confidence 必须位于 [0,1]");

  valid &= require(finite_scalar(config_.q_cv) && config_.q_cv >= 0.0,
                   "q_cv 必须为非负有限值");
  valid &= require(finite_scalar(config_.q_ca) && config_.q_ca >= 0.0,
                   "q_ca 必须为非负有限值");
  valid &= require(finite_scalar(config_.q_acceleration) && config_.q_acceleration >= 0.0,
                   "q_acceleration 必须为非负有限值");
  valid &= require(finite_scalar(config_.cv_acceleration_retention) &&
                       config_.cv_acceleration_retention >= 0.0 &&
                       config_.cv_acceleration_retention <= 1.0,
                   "cv_acceleration_retention 必须位于 [0,1]");
  valid &= require(finite_scalar(config_.transition_stay_probability) &&
                       config_.transition_stay_probability > 0.0 &&
                       config_.transition_stay_probability < 1.0,
                   "transition_stay_probability 必须位于 (0,1)");
  valid &= require(finite_scalar(config_.initial_cv_probability) &&
                       config_.initial_cv_probability > 0.0 && config_.initial_cv_probability < 1.0,
                   "initial_cv_probability 必须位于 (0,1)");
  valid &= require(finite_scalar(config_.initial_position_variance) && config_.initial_position_variance > 0.0,
                   "initial_position_variance 必须大于 0");
  valid &= require(finite_scalar(config_.initial_velocity_variance) && config_.initial_velocity_variance > 0.0,
                   "initial_velocity_variance 必须大于 0");
  valid &= require(finite_scalar(config_.initial_acceleration_variance) &&
                       config_.initial_acceleration_variance > 0.0,
                   "initial_acceleration_variance 必须大于 0");

  valid &= require(finite_scalar(config_.measurement_variance) && config_.measurement_variance > 0.0,
                   "measurement_variance 必须大于 0");
  valid &= require(finite_scalar(config_.association_epsilon) && config_.association_epsilon > 0.0,
                   "association_epsilon 必须大于 0");
  valid &= require(finite_scalar(config_.empty_weight) && config_.empty_weight > 0.0,
                   "empty_weight 必须大于 0");
  valid &= require(finite_scalar(config_.birth_confidence) && config_.birth_confidence >= 0.0,
                   "birth_confidence 必须为非负有限值");
  valid &= require(finite_scalar(config_.hit_beta_empty_threshold) &&
                       config_.hit_beta_empty_threshold > 0.0 && config_.hit_beta_empty_threshold < 1.0,
                   "hit_beta_empty_threshold 必须位于 (0,1)");
  valid &= require(finite_scalar(config_.tentative_visual_association_ratio) &&
                       config_.tentative_visual_association_ratio >= 0.0 &&
                       config_.tentative_visual_association_ratio <= 1.0,
                   "tentative_visual_association_ratio 必须位于 [0,1]");
  valid &= require(config_.hits_to_confirm >= 1, "hits_to_confirm 必须至少为 1");
  valid &= require(config_.tentative_misses_to_delete >= 1,
                   "tentative_misses_to_delete 必须至少为 1");
  valid &= require(config_.confirmed_misses_to_lost >= 1,
                   "confirmed_misses_to_lost 必须至少为 1");
  valid &= require(config_.lost_misses_to_delete >= config_.confirmed_misses_to_lost,
                   "lost_misses_to_delete 不得小于 confirmed_misses_to_lost");
  valid &= require(finite_scalar(config_.lost_covariance_inflation) &&
                       config_.lost_covariance_inflation >= 1.0,
                   "lost_covariance_inflation 必须至少为 1");
  valid &= require(finite_scalar(config_.maximum_frame_gap_s) && config_.maximum_frame_gap_s > 0.0,
                   "maximum_frame_gap_s 必须大于 0");

  valid &= require(finite_scalar(camera_.fx) && camera_.fx > 0.0 &&
                       finite_scalar(camera_.fy) && camera_.fy > 0.0,
                   "相机焦距必须为正有限值");
  valid &= require(finite_scalar(camera_.cx) && finite_scalar(camera_.cy),
                   "相机主点必须为有限值");
  valid &= require(finite_scalar(camera_.k1) && finite_scalar(camera_.k2) &&
                       finite_scalar(camera_.p1) && finite_scalar(camera_.p2) && finite_scalar(camera_.k3),
                   "相机畸变参数必须为有限值");
  valid &= require(camera_.image_width > 0 && camera_.image_height > 0,
                   "相机图像尺寸必须为正数");
  return valid;
}

bool IMMPDAFTracker::project_world_point(const Eigen::Vector3d& point_world,
                                         const Eigen::Matrix4d& T_C_W,
                                         Eigen::Vector2d& pixel,
                                         double* range_m) const
{
  pixel = Eigen::Vector2d::Constant(-1.0);
  if (!point_world.allFinite() || !T_C_W.allFinite())
  {
    return false;
  }

  Eigen::Vector4d homogeneous(point_world.x(), point_world.y(), point_world.z(), 1.0);
  Eigen::Vector4d camera_homogeneous = T_C_W * homogeneous;
  if (!camera_homogeneous.allFinite() || std::abs(camera_homogeneous.w()) < MINIMUM_DEPTH_M)
  {
    return false;
  }
  Eigen::Vector3d point_camera = camera_homogeneous.head<3>() / camera_homogeneous.w();
  if (range_m != nullptr)
  {
    *range_m = point_camera.norm();
  }
  if (!point_camera.allFinite() || point_camera.z() <= MINIMUM_DEPTH_M)
  {
    return false;
  }

  const double x = point_camera.x() / point_camera.z();
  const double y = point_camera.y() / point_camera.z();
  const double ideal_u = camera_.fx * x + camera_.cx;
  const double ideal_v = camera_.fy * y + camera_.cy;
  const double horizontal_margin = IDEAL_IMAGE_MARGIN_RATIO * camera_.image_width;
  const double vertical_margin = IDEAL_IMAGE_MARGIN_RATIO * camera_.image_height;
  if (!finite_scalar(ideal_u) || !finite_scalar(ideal_v)
      || ideal_u < -horizontal_margin || ideal_u >= camera_.image_width + horizontal_margin
      || ideal_v < -vertical_margin || ideal_v >= camera_.image_height + vertical_margin)
  {
    return false;
  }

  const double r2 = x * x + y * y;
  const double radial = 1.0 + camera_.k1 * r2 + camera_.k2 * r2 * r2 + camera_.k3 * r2 * r2 * r2;
  const double distorted_x = x * radial + 2.0 * camera_.p1 * x * y + camera_.p2 * (r2 + 2.0 * x * x);
  const double distorted_y = y * radial + camera_.p1 * (r2 + 2.0 * y * y) + 2.0 * camera_.p2 * x * y;
  pixel.x() = camera_.fx * distorted_x + camera_.cx;
  pixel.y() = camera_.fy * distorted_y + camera_.cy;
  if (!pixel.allFinite())
  {
    pixel = Eigen::Vector2d::Constant(-1.0);
    return false;
  }

  return pixel.x() >= 0.0 && pixel.x() < static_cast<double>(camera_.image_width) &&
         pixel.y() >= 0.0 && pixel.y() < static_cast<double>(camera_.image_height);
}

std::vector<Candidate3D> IMMPDAFTracker::generate_candidates(const TrackingPointCloud& cloud_world,
                                                              const DetectionFrame* detection,
                                                              const Eigen::Matrix4d& T_C_W,
                                                              const MotionPrior& motion_prior,
                                                              std::string& diagnostic) const
{
  std::vector<const Detection2D*> target_detections;
  if (detection != nullptr)
  {
    target_detections.reserve(detection->detections.size());
    for (const auto& item : detection->detections)
    {
      if (config_.target_class_id >= 0 && item.class_id != config_.target_class_id)
      {
        continue;
      }
      if (!finite_scalar(item.confidence) ||
          item.confidence < config_.minimum_detection_confidence ||
          !finite_scalar(item.x1) || !finite_scalar(item.y1) ||
          !finite_scalar(item.x2) || !finite_scalar(item.y2))
      {
        continue;
      }
      target_detections.push_back(&item);
    }
  }

  const bool transform_valid = T_C_W.allFinite();
  if (!transform_valid)
  {
    append_diagnostic(diagnostic, "T_C_W 非有限，当前帧禁用视觉候选");
  }

  Eigen::Matrix3d motion_covariance = Eigen::Matrix3d::Identity();
  Eigen::LDLT<Eigen::Matrix3d> motion_ldlt;
  bool motion_gate_valid = motion_prior.valid && motion_prior.position.allFinite() &&
                           motion_prior.covariance.allFinite();
  if (motion_gate_valid)
  {
    motion_covariance = 0.5 * (motion_prior.covariance + motion_prior.covariance.transpose());
    motion_covariance.diagonal().array() +=
        config_.motion_expansion_sigma * config_.motion_expansion_sigma;
    motion_ldlt.compute(motion_covariance);
    if (motion_ldlt.info() != Eigen::Success ||
        (motion_ldlt.vectorD().array() <= MINIMUM_VARIANCE).any())
    {
      motion_covariance.diagonal().array() += MINIMUM_VARIANCE;
      motion_ldlt.compute(motion_covariance);
    }
    motion_gate_valid = motion_ldlt.info() == Eigen::Success &&
                        (motion_ldlt.vectorD().array() > 0.0).all();
    if (!motion_gate_valid)
    {
      append_diagnostic(diagnostic, "运动门协方差分解失败，当前帧仅使用视觉 ROI");
    }
  }

  std::vector<ProjectedPoint> selected_points;
  selected_points.reserve(cloud_world.points.size() / 8 + 1);
  const double visual_bbox_scale_x =
      motion_gate_valid ? config_.visual_association_bbox_scale_x : 1.0;
  const double visual_bbox_scale_y =
      motion_gate_valid ? config_.visual_association_bbox_scale_y : 1.0;
  for (const auto& point : cloud_world.points)
  {
    Eigen::Vector3d point_world(static_cast<double>(point.x),
                                static_cast<double>(point.y),
                                static_cast<double>(point.z));
    if (!point_world.allFinite())
    {
      continue;
    }
    if (point_world.z() < config_.minimum_height_m)
    {
      continue;
    }

    Eigen::Vector2d pixel = Eigen::Vector2d::Constant(-1.0);
    double range_m = point_world.norm();
    const bool projection_valid = transform_valid && project_world_point(point_world, T_C_W, pixel, &range_m);
    if (!finite_scalar(range_m) || range_m < config_.minimum_range_m || range_m > config_.maximum_range_m)
    {
      continue;
    }

    bool from_motion = false;
    if (motion_gate_valid)
    {
      const Eigen::Vector3d difference = point_world - motion_prior.position;
      const Eigen::Vector3d solved = motion_ldlt.solve(difference);
      const double squared_distance = difference.dot(solved);
      from_motion = finite_scalar(squared_distance) && squared_distance >= 0.0 &&
                    squared_distance < config_.motion_gate_threshold;
    }

    bool from_visual_roi = false;
    bool from_visual_association = false;
    if (projection_valid)
    {
      for (const Detection2D* item : target_detections)
      {
        const double x_min = std::min(item->x1, item->x2);
        const double x_max = std::max(item->x1, item->x2);
        const double y_min = std::min(item->y1, item->y2);
        const double y_max = std::max(item->y1, item->y2);
        const double center_x = 0.5 * (x_min + x_max);
        const double center_y = 0.5 * (y_min + y_max);
        const double half_width = 0.5 * (x_max - x_min) * visual_bbox_scale_x;
        const double half_height = 0.5 * (y_max - y_min) * visual_bbox_scale_y;
        const bool inside_original_bbox =
            pixel.x() >= x_min && pixel.x() <= x_max && pixel.y() >= y_min && pixel.y() <= y_max;
        if (inside_original_bbox)
        {
          from_visual_roi = true;
          from_visual_association = true;
          break;
        }
        const bool inside_association_bbox =
            pixel.x() >= center_x - half_width && pixel.x() <= center_x + half_width &&
            pixel.y() >= center_y - half_height && pixel.y() <= center_y + half_height;
        if (from_motion && inside_association_bbox)
        {
          from_visual_association = true;
        }
      }
    }

    if (from_motion || from_visual_roi)
    {
      ProjectedPoint selected;
      selected.world = point_world;
      selected.pixel = pixel;
      selected.projection_valid = projection_valid;
      selected.from_motion = from_motion;
      selected.from_visual = from_visual_association;
      selected_points.push_back(selected);
    }
  }

  if (selected_points.empty())
  {
    return {};
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  roi_cloud->points.reserve(selected_points.size());
  for (const auto& selected : selected_points)
  {
    roi_cloud->points.emplace_back(static_cast<float>(selected.world.x()),
                                   static_cast<float>(selected.world.y()),
                                   static_cast<float>(selected.world.z()));
  }
  roi_cloud->width = static_cast<uint32_t>(roi_cloud->points.size());
  roi_cloud->height = 1;
  roi_cloud->is_dense = true;

  std::vector<pcl::PointIndices> clusters;
  try
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(roi_cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
    extraction.setClusterTolerance(config_.cluster_tolerance_m);
    extraction.setMinClusterSize(std::max(1, config_.cluster_min_points));
    extraction.setMaxClusterSize(static_cast<int>(std::min<size_t>(
        roi_cloud->points.size(), static_cast<size_t>(std::numeric_limits<int>::max()))));
    extraction.setSearchMethod(tree);
    extraction.setInputCloud(roi_cloud);
    extraction.extract(clusters);
  }
  catch (const std::exception& error)
  {
    append_diagnostic(diagnostic, std::string("欧氏聚类失败: ") + error.what());
    return {};
  }

  std::vector<Candidate3D> candidates;
  candidates.reserve(clusters.size());
  for (const auto& cluster : clusters)
  {
    if (cluster.indices.empty())
    {
      continue;
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    bool from_motion = false;
    bool from_visual = false;
    for (int index : cluster.indices)
    {
      if (index < 0 || static_cast<size_t>(index) >= selected_points.size())
      {
        continue;
      }
      const auto& selected = selected_points[static_cast<size_t>(index)];
      centroid += selected.world;
      from_motion = from_motion || selected.from_motion;
      from_visual = from_visual || selected.from_visual;
    }
    centroid /= static_cast<double>(cluster.indices.size());
    if (!centroid.allFinite())
    {
      continue;
    }

    Eigen::Vector2d centroid_pixel = Eigen::Vector2d::Constant(-1.0);
    const bool centroid_projection_valid =
        transform_valid && project_world_point(centroid, T_C_W, centroid_pixel, nullptr);
    double best_visual_score = 0.0;
    if (centroid_projection_valid)
    {
      for (const Detection2D* item : target_detections)
      {
        const double x_min = std::min(item->x1, item->x2);
        const double x_max = std::max(item->x1, item->x2);
        const double y_min = std::min(item->y1, item->y2);
        const double y_max = std::max(item->y1, item->y2);
        if (centroid_pixel.x() < x_min - VISUAL_SCORE_BBOX_MARGIN_PX ||
            centroid_pixel.x() > x_max + VISUAL_SCORE_BBOX_MARGIN_PX ||
            centroid_pixel.y() < y_min - VISUAL_SCORE_BBOX_MARGIN_PX ||
            centroid_pixel.y() > y_max + VISUAL_SCORE_BBOX_MARGIN_PX)
        {
          continue;
        }
        const double width = std::max(x_max - x_min, 1.0);
        const double height = std::max(y_max - y_min, 1.0);
        const double normalized_x = (centroid_pixel.x() - 0.5 * (x_min + x_max)) / width;
        const double normalized_y = (centroid_pixel.y() - 0.5 * (y_min + y_max)) / height;
        const double squared_normalized_distance =
            normalized_x * normalized_x + normalized_y * normalized_y;
        const double decay = std::exp(-squared_normalized_distance /
                                      (2.0 * config_.visual_sigma * config_.visual_sigma));
        best_visual_score = std::max(best_visual_score, item->confidence * decay);
      }
    }

    Candidate3D candidate;
    candidate.position = centroid;
    candidate.visual_confidence = config_.visual_baseline + best_visual_score;
    candidate.point_count = static_cast<int>(std::min<size_t>(
        cluster.indices.size(), static_cast<size_t>(std::numeric_limits<int>::max())));
    candidate.from_motion = from_motion;
    candidate.from_visual = from_visual;
    candidate.projected_pixel = centroid_projection_valid
                                    ? centroid_pixel
                                    : Eigen::Vector2d::Constant(-1.0);
    candidates.push_back(candidate);
  }
  return candidates;
}

void IMMPDAFTracker::initialize_track(const Eigen::Vector3d& position)
{
  TrackingState9 state = TrackingState9::Zero();
  state.head<3>() = position;
  TrackingCovariance9 covariance = TrackingCovariance9::Zero();
  covariance.diagonal().segment<3>(0).setConstant(config_.initial_position_variance);
  covariance.diagonal().segment<3>(3).setConstant(config_.initial_velocity_variance);
  covariance.diagonal().segment<3>(6).setConstant(config_.initial_acceleration_variance);
  for (auto& mode : modes_)
  {
    mode.state = state;
    mode.covariance = covariance;
  }
  mode_probabilities_ = Eigen::Vector2d(config_.initial_cv_probability,
                                        1.0 - config_.initial_cv_probability);
  initialized_ = true;
  lifecycle_ = TrackLifecycle::TENTATIVE;
  hits_ = 1;
  misses_ = 0;
  lost_duration_s_ = 0.0;
}

bool IMMPDAFTracker::predict_modes(double dt,
                                   std::array<ModeTrackState, MODE_COUNT>& predicted_modes,
                                   Eigen::Vector2d& predicted_probabilities,
                                   std::string& diagnostic) const
{
  if (!initialized_ || !finite_scalar(dt) || dt <= 0.0)
  {
    append_diagnostic(diagnostic, "活动轨迹的 dt 必须为正有限值");
    return false;
  }

  const double stay = config_.transition_stay_probability;
  Eigen::Matrix2d transition;
  transition << stay, 1.0 - stay,
                1.0 - stay, stay;
  predicted_probabilities = transition * mode_probabilities_;
  const double probability_sum = predicted_probabilities.sum();
  if (!predicted_probabilities.allFinite() || probability_sum <= 0.0)
  {
    append_diagnostic(diagnostic, "IMM 先验模式概率无效");
    return false;
  }
  predicted_probabilities /= probability_sum;

  std::array<ModeTrackState, MODE_COUNT> mixed_modes;
  for (int destination = 0; destination < MODE_COUNT; ++destination)
  {
    Eigen::Vector2d mixing_probabilities = Eigen::Vector2d::Zero();
    for (int source = 0; source < MODE_COUNT; ++source)
    {
      mixing_probabilities(source) = transition(destination, source) * mode_probabilities_(source) /
                                     predicted_probabilities(destination);
    }
    const double mixing_sum = mixing_probabilities.sum();
    if (!mixing_probabilities.allFinite() || mixing_sum <= 0.0)
    {
      append_diagnostic(diagnostic, "IMM 混合概率无效");
      return false;
    }
    mixing_probabilities /= mixing_sum;

    mixed_modes[destination].state.setZero();
    for (int source = 0; source < MODE_COUNT; ++source)
    {
      mixed_modes[destination].state += mixing_probabilities(source) * modes_[source].state;
    }
    mixed_modes[destination].covariance.setZero();
    for (int source = 0; source < MODE_COUNT; ++source)
    {
      const TrackingState9 difference = modes_[source].state - mixed_modes[destination].state;
      mixed_modes[destination].covariance += mixing_probabilities(source) *
          (modes_[source].covariance + difference * difference.transpose());
    }
    if (!stabilize_covariance(mixed_modes[destination].covariance))
    {
      append_diagnostic(diagnostic, "IMM 混合协方差无效");
      return false;
    }
  }

  std::array<TrackingCovariance9, MODE_COUNT> transitions;
  std::array<TrackingCovariance9, MODE_COUNT> process_covariances;
  for (auto& matrix : transitions)
  {
    matrix.setIdentity();
  }
  for (auto& matrix : process_covariances)
  {
    matrix.setZero();
  }

  transitions[CV_MODE].block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
  transitions[CV_MODE].block<3, 3>(6, 6) =
      config_.cv_acceleration_retention * Eigen::Matrix3d::Identity();
  transitions[CA_MODE].block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
  transitions[CA_MODE].block<3, 3>(0, 6) = 0.5 * dt * dt * Eigen::Matrix3d::Identity();
  transitions[CA_MODE].block<3, 3>(3, 6) = dt * Eigen::Matrix3d::Identity();

  for (int axis = 0; axis < 3; ++axis)
  {
    process_covariances[CV_MODE](axis, axis) = dt * dt * dt * config_.q_cv / 3.0;
    process_covariances[CV_MODE](axis, axis + 3) = dt * dt * config_.q_cv / 2.0;
    process_covariances[CV_MODE](axis + 3, axis) = dt * dt * config_.q_cv / 2.0;
    process_covariances[CV_MODE](axis + 3, axis + 3) = dt * config_.q_cv;
    process_covariances[CV_MODE](axis + 6, axis + 6) = config_.q_acceleration;

    const std::array<int, 3> indices = {axis, axis + 3, axis + 6};
    Eigen::Matrix3d ca_block;
    ca_block << std::pow(dt, 5) / 20.0, std::pow(dt, 4) / 8.0, std::pow(dt, 3) / 6.0,
                std::pow(dt, 4) / 8.0, std::pow(dt, 3) / 3.0, dt * dt / 2.0,
                std::pow(dt, 3) / 6.0, dt * dt / 2.0, dt;
    ca_block *= config_.q_ca;
    for (int row = 0; row < 3; ++row)
    {
      for (int column = 0; column < 3; ++column)
      {
        process_covariances[CA_MODE](indices[row], indices[column]) = ca_block(row, column);
      }
    }
  }

  for (int mode = 0; mode < MODE_COUNT; ++mode)
  {
    predicted_modes[mode].state = transitions[mode] * mixed_modes[mode].state;
    predicted_modes[mode].covariance = transitions[mode] * mixed_modes[mode].covariance *
                                       transitions[mode].transpose() + process_covariances[mode];
    if (!predicted_modes[mode].state.allFinite() ||
        !stabilize_covariance(predicted_modes[mode].covariance))
    {
      append_diagnostic(diagnostic, "IMM 模式预测出现非有限数值");
      return false;
    }
  }
  return true;
}

bool IMMPDAFTracker::update_mode(const ModeTrackState& predicted,
                                 const std::vector<Candidate3D>& candidates,
                                 ModeUpdate& update,
                                 std::string& diagnostic) const
{
  update.posterior = predicted;
  update.association_probabilities.assign(candidates.size() + 1, 0.0);
  if (candidates.empty())
  {
    update.association_probabilities[0] = 1.0;
    update.log_evidence = std::log(config_.empty_weight);
    return true;
  }

  const Eigen::Matrix<double, 9, 3> covariance_position = predicted.covariance.block<9, 3>(0, 0);
  Eigen::Matrix3d innovation_covariance = predicted.covariance.block<3, 3>(0, 0) +
                                          config_.measurement_variance * Eigen::Matrix3d::Identity();
  innovation_covariance = 0.5 * (innovation_covariance + innovation_covariance.transpose());

  Eigen::LDLT<Eigen::Matrix3d> innovation_ldlt;
  double jitter = MINIMUM_VARIANCE;
  bool factorized = false;
  for (int attempt = 0; attempt < 6; ++attempt)
  {
    innovation_ldlt.compute(innovation_covariance);
    if (innovation_ldlt.info() == Eigen::Success &&
        (innovation_ldlt.vectorD().array() > MINIMUM_VARIANCE * 0.1).all())
    {
      factorized = true;
      break;
    }
    innovation_covariance.diagonal().array() += jitter;
    jitter *= 10.0;
  }
  if (!factorized)
  {
    append_diagnostic(diagnostic, "PDAF 新息协方差 LDLT 分解失败");
    return false;
  }

  const Eigen::Matrix<double, 9, 3> gain =
      innovation_ldlt.solve(covariance_position.transpose()).transpose();
  if (!gain.allFinite())
  {
    append_diagnostic(diagnostic, "PDAF Kalman 增益非有限");
    return false;
  }

  double log_determinant = 0.0;
  for (int axis = 0; axis < 3; ++axis)
  {
    const double diagonal = innovation_ldlt.vectorD()(axis);
    if (!finite_scalar(diagonal) || diagonal <= 0.0)
    {
      append_diagnostic(diagnostic, "PDAF 新息协方差行列式无效");
      return false;
    }
    log_determinant += std::log(diagonal);
  }

  std::vector<Eigen::Vector3d> innovations;
  innovations.reserve(candidates.size());
  std::vector<double> log_weights;
  log_weights.reserve(candidates.size() + 1);
  log_weights.push_back(std::log(config_.empty_weight));
  for (const auto& candidate : candidates)
  {
    const Eigen::Vector3d innovation = candidate.position - predicted.state.head<3>();
    const Eigen::Vector3d solved = innovation_ldlt.solve(innovation);
    double normalized_innovation = innovation.dot(solved);
    if (!finite_scalar(normalized_innovation))
    {
      append_diagnostic(diagnostic, "PDAF 马氏距离非有限");
      return false;
    }
    normalized_innovation = std::max(0.0, normalized_innovation);
    const double log_gaussian = -0.5 * (3.0 * LOG_TWO_PI + log_determinant + normalized_innovation);
    const double visual_weight = config_.association_epsilon +
                                 std::max(0.0, candidate.visual_confidence);
    log_weights.push_back(log_gaussian + std::log(visual_weight));
    innovations.push_back(innovation);
  }

  const double maximum_log_weight = *std::max_element(log_weights.begin(), log_weights.end());
  double scaled_sum = 0.0;
  for (double log_weight : log_weights)
  {
    scaled_sum += std::exp(log_weight - maximum_log_weight);
  }
  if (!finite_scalar(maximum_log_weight) || !finite_scalar(scaled_sum) || scaled_sum <= 0.0)
  {
    append_diagnostic(diagnostic, "PDAF 关联权重归一化失败");
    return false;
  }
  update.log_evidence = maximum_log_weight + std::log(scaled_sum);
  for (size_t index = 0; index < log_weights.size(); ++index)
  {
    update.association_probabilities[index] =
        std::exp(log_weights[index] - update.log_evidence);
  }
  double association_sum = 0.0;
  for (double probability : update.association_probabilities)
  {
    association_sum += probability;
  }
  if (!finite_scalar(association_sum) || association_sum <= 0.0)
  {
    append_diagnostic(diagnostic, "PDAF 关联概率无效");
    return false;
  }
  for (double& probability : update.association_probabilities)
  {
    probability /= association_sum;
  }

  Eigen::Vector3d weighted_innovation = Eigen::Vector3d::Zero();
  Eigen::Matrix3d innovation_second_moment = Eigen::Matrix3d::Zero();
  for (size_t index = 0; index < innovations.size(); ++index)
  {
    const double probability = update.association_probabilities[index + 1];
    weighted_innovation += probability * innovations[index];
    innovation_second_moment += probability * innovations[index] * innovations[index].transpose();
  }

  update.posterior.state = predicted.state + gain * weighted_innovation;
  const double beta_empty = update.association_probabilities.front();
  const Eigen::Matrix3d association_spread =
      innovation_second_moment - weighted_innovation * weighted_innovation.transpose();
  update.posterior.covariance = predicted.covariance -
      (1.0 - beta_empty) * gain * innovation_covariance * gain.transpose() +
      gain * association_spread * gain.transpose();
  if (!update.posterior.state.allFinite() ||
      !stabilize_covariance(update.posterior.covariance))
  {
    append_diagnostic(diagnostic, "PDAF 后验状态或协方差无效");
    return false;
  }
  return true;
}

bool IMMPDAFTracker::fuse_modes(const std::array<ModeTrackState, MODE_COUNT>& modes,
                                const Eigen::Vector2d& probabilities,
                                TrackingState9& state,
                                TrackingCovariance9& covariance) const
{
  if (!all_state_finite(modes, probabilities))
  {
    return false;
  }
  Eigen::Vector2d normalized_probabilities = probabilities / probabilities.sum();
  state.setZero();
  for (int mode = 0; mode < MODE_COUNT; ++mode)
  {
    state += normalized_probabilities(mode) * modes[mode].state;
  }
  covariance.setZero();
  for (int mode = 0; mode < MODE_COUNT; ++mode)
  {
    const TrackingState9 difference = modes[mode].state - state;
    covariance += normalized_probabilities(mode) *
                  (modes[mode].covariance + difference * difference.transpose());
  }
  return state.allFinite() && stabilize_covariance(covariance);
}

bool IMMPDAFTracker::stabilize_covariance(TrackingCovariance9& covariance) const
{
  if (!covariance.allFinite())
  {
    return false;
  }
  covariance = 0.5 * (covariance + covariance.transpose());
  Eigen::SelfAdjointEigenSolver<TrackingCovariance9> solver(covariance, Eigen::EigenvaluesOnly);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite())
  {
    return false;
  }
  const double minimum_eigenvalue = solver.eigenvalues().minCoeff();
  if (minimum_eigenvalue < MINIMUM_VARIANCE)
  {
    covariance.diagonal().array() += MINIMUM_VARIANCE - minimum_eigenvalue;
    covariance = 0.5 * (covariance + covariance.transpose());
  }
  return covariance.allFinite();
}

bool IMMPDAFTracker::all_state_finite(const std::array<ModeTrackState, MODE_COUNT>& modes,
                                      const Eigen::Vector2d& probabilities) const
{
  if (!probabilities.allFinite() || (probabilities.array() < 0.0).any() || probabilities.sum() <= 0.0)
  {
    return false;
  }
  for (const auto& mode : modes)
  {
    if (!mode.state.allFinite() || !mode.covariance.allFinite())
    {
      return false;
    }
  }
  return true;
}

TrackResult IMMPDAFTracker::make_result(
    int64_t timestamp_us,
    const std::vector<Candidate3D>& candidates,
    double beta_empty,
    const std::array<std::vector<double>, MODE_COUNT>& associations,
    bool numerical_ok,
    const std::string& diagnostic) const
{
  TrackResult result;
  result.frame_timestamp_us = timestamp_us;
  result.lifecycle = lifecycle_;
  result.mode_probabilities = mode_probabilities_;
  result.beta_empty = beta_empty;
  result.hits = hits_;
  result.misses = misses_;
  result.lost_duration_s = lost_duration_s_;
  result.candidates = candidates;
  result.mode_association_probabilities = associations;
  result.numerical_ok = numerical_ok;
  result.diagnostic = diagnostic;

  if (!initialized_)
  {
    result.state_valid = false;
    return result;
  }

  result.modes = modes_;
  TrackingState9 fused_state;
  TrackingCovariance9 fused_covariance;
  if (!fuse_modes(modes_, mode_probabilities_, fused_state, fused_covariance))
  {
    result.state_valid = false;
    result.numerical_ok = false;
    append_diagnostic(result.diagnostic, "IMM 后验融合失败");
    return result;
  }
  result.state = fused_state;
  result.covariance = fused_covariance;
  result.state_valid = true;
  return result;
}

TrackResult IMMPDAFTracker::step(const TrackingPointCloud& cloud_world,
                                 const DetectionFrame* detection,
                                 const Eigen::Matrix4d& T_C_W,
                                 int64_t timestamp_us,
                                 double dt)
{
  std::array<std::vector<double>, MODE_COUNT> associations;
  associations[CV_MODE] = {1.0};
  associations[CA_MODE] = {1.0};
  if (!configuration_valid_)
  {
    reset();
    return make_result(timestamp_us, {}, 1.0, associations, false,
                       std::string("跟踪配置无效: ") + configuration_diagnostic_);
  }

  std::string diagnostic;
  if (initialized_ && (!finite_scalar(dt) || dt <= 0.0))
  {
    reset();
    return make_result(timestamp_us, {}, 1.0, associations, false,
                       "活动轨迹收到非正或非有限 dt，已重置");
  }
  if (initialized_ && dt > config_.maximum_frame_gap_s)
  {
    reset();
    append_diagnostic(diagnostic, "帧间隔超过 maximum_frame_gap_s，已重置并重新等待视觉起始");
  }

  std::array<ModeTrackState, MODE_COUNT> predicted_modes;
  Eigen::Vector2d predicted_probabilities = mode_probabilities_;
  MotionPrior motion_prior;
  if (initialized_)
  {
    if (!predict_modes(dt, predicted_modes, predicted_probabilities, diagnostic))
    {
      reset();
      return make_result(timestamp_us, {}, 1.0, associations, false, diagnostic);
    }

    TrackingState9 predicted_state;
    TrackingCovariance9 predicted_covariance;
    if (!fuse_modes(predicted_modes, predicted_probabilities, predicted_state, predicted_covariance))
    {
      append_diagnostic(diagnostic, "IMM 融合预测无效，已重置");
      reset();
      return make_result(timestamp_us, {}, 1.0, associations, false, diagnostic);
    }
    motion_prior.valid = true;
    motion_prior.position = predicted_state.head<3>();
    motion_prior.covariance = predicted_covariance.block<3, 3>(0, 0);
  }

  std::vector<Candidate3D> candidates =
      generate_candidates(cloud_world, detection, T_C_W, motion_prior, diagnostic);

  if (!initialized_)
  {
    size_t selected_index = candidates.size();
    double selected_confidence = -std::numeric_limits<double>::infinity();
    for (size_t index = 0; index < candidates.size(); ++index)
    {
      const Candidate3D& candidate = candidates[index];
      if (candidate.from_visual && candidate.visual_confidence >= config_.birth_confidence &&
          candidate.visual_confidence > selected_confidence)
      {
        selected_index = index;
        selected_confidence = candidate.visual_confidence;
      }
    }
    if (selected_index == candidates.size())
    {
      return make_result(timestamp_us, candidates, 1.0, associations, true, diagnostic);
    }

    initialize_track(candidates[selected_index].position);
    associations[CV_MODE].assign(candidates.size() + 1, 0.0);
    associations[CA_MODE].assign(candidates.size() + 1, 0.0);
    associations[CV_MODE][selected_index + 1] = 1.0;
    associations[CA_MODE][selected_index + 1] = 1.0;
    return make_result(timestamp_us, candidates, 0.0, associations, true, diagnostic);
  }

  const bool was_lost = lifecycle_ == TrackLifecycle::LOST;
  modes_ = predicted_modes;
  mode_probabilities_ = predicted_probabilities;
  double fused_beta_empty = 1.0;
  if (candidates.empty())
  {
    associations[CV_MODE] = {1.0};
    associations[CA_MODE] = {1.0};
  }
  else
  {
    std::array<ModeUpdate, MODE_COUNT> updates;
    for (int mode = 0; mode < MODE_COUNT; ++mode)
    {
      if (!update_mode(predicted_modes[mode], candidates, updates[mode], diagnostic))
      {
        reset();
        return make_result(timestamp_us, candidates, 1.0, associations, false, diagnostic);
      }
    }

    Eigen::Vector2d log_mode_weights;
    for (int mode = 0; mode < MODE_COUNT; ++mode)
    {
      log_mode_weights(mode) = std::log(std::max(predicted_probabilities(mode),
                                                  std::numeric_limits<double>::min())) +
                               updates[mode].log_evidence;
    }
    const double maximum_log_mode_weight = log_mode_weights.maxCoeff();
    Eigen::Vector2d scaled_mode_weights =
        (log_mode_weights.array() - maximum_log_mode_weight).exp().matrix();
    const double mode_weight_sum = scaled_mode_weights.sum();
    if (!scaled_mode_weights.allFinite() || !finite_scalar(mode_weight_sum) || mode_weight_sum <= 0.0)
    {
      append_diagnostic(diagnostic, "IMM 模式证据归一化失败，已重置");
      reset();
      return make_result(timestamp_us, candidates, 1.0, associations, false, diagnostic);
    }
    mode_probabilities_ = scaled_mode_weights / mode_weight_sum;
    for (int mode = 0; mode < MODE_COUNT; ++mode)
    {
      modes_[mode] = updates[mode].posterior;
      associations[mode] = updates[mode].association_probabilities;
      fused_beta_empty += mode_probabilities_(mode) * associations[mode].front();
    }
    fused_beta_empty -= 1.0;
  }

  if (!all_state_finite(modes_, mode_probabilities_) || !finite_scalar(fused_beta_empty))
  {
    append_diagnostic(diagnostic, "跟踪后验出现非有限数值，已重置");
    reset();
    return make_result(timestamp_us, candidates, 1.0, associations, false, diagnostic);
  }
  fused_beta_empty = std::clamp(fused_beta_empty, 0.0, 1.0);

  const double non_empty_association = 1.0 - fused_beta_empty;
  double visual_association = 0.0;
  if (non_empty_association > std::numeric_limits<double>::epsilon())
  {
    for (size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index)
    {
      const Candidate3D& candidate = candidates[candidate_index];
      const bool strong_visual_candidate = candidate.from_visual
          && candidate.visual_confidence >= config_.birth_confidence
          && candidate.projected_pixel.allFinite()
          && candidate.projected_pixel.x() >= 0.0
          && candidate.projected_pixel.x() < camera_.image_width
          && candidate.projected_pixel.y() >= 0.0
          && candidate.projected_pixel.y() < camera_.image_height;
      if (!strong_visual_candidate)
      {
        continue;
      }
      for (int mode = 0; mode < MODE_COUNT; ++mode)
      {
        if (associations[mode].size() == candidates.size() + 1)
        {
          visual_association += mode_probabilities_(mode) * associations[mode][candidate_index + 1];
        }
      }
    }
  }
  const double visual_association_ratio = non_empty_association > std::numeric_limits<double>::epsilon()
      ? std::clamp(visual_association / non_empty_association, 0.0, 1.0)
      : 0.0;
  const bool association_hit = !candidates.empty()
      && fused_beta_empty < config_.hit_beta_empty_threshold;
  const bool tentative_visual_hit = lifecycle_ != TrackLifecycle::TENTATIVE
      || visual_association_ratio >= config_.tentative_visual_association_ratio;
  const bool hit = association_hit && tentative_visual_hit;
  if (association_hit && !tentative_visual_hit)
  {
    append_diagnostic(diagnostic, "tentative 命中缺少足够视觉关联");
  }
  if (was_lost && !hit)
  {
    modes_ = predicted_modes;
    mode_probabilities_ = predicted_probabilities;
  }
  if (hit)
  {
    ++hits_;
    misses_ = 0;
    lost_duration_s_ = 0.0;
    if (lifecycle_ == TrackLifecycle::TENTATIVE && hits_ >= config_.hits_to_confirm)
    {
      lifecycle_ = TrackLifecycle::CONFIRMED;
    }
    else if (lifecycle_ == TrackLifecycle::LOST)
    {
      lifecycle_ = TrackLifecycle::CONFIRMED;
    }
  }
  else
  {
    hits_ = 0;
    ++misses_;
    if (lifecycle_ == TrackLifecycle::TENTATIVE && misses_ >= config_.tentative_misses_to_delete)
    {
      append_diagnostic(diagnostic, "tentative 连续 miss 达到阈值，轨迹已删除");
      reset();
    }
    else if (lifecycle_ == TrackLifecycle::CONFIRMED && misses_ >= config_.confirmed_misses_to_lost)
    {
      lifecycle_ = TrackLifecycle::LOST;
      lost_duration_s_ = 0.0;
    }
    else if (lifecycle_ == TrackLifecycle::LOST)
    {
      lost_duration_s_ += dt;
      for (auto& mode : modes_)
      {
        mode.covariance *= config_.lost_covariance_inflation;
        if (!stabilize_covariance(mode.covariance))
        {
          append_diagnostic(diagnostic, "lost 协方差膨胀失败，轨迹已重置");
          reset();
          return make_result(timestamp_us, candidates, 1.0, associations, false, diagnostic);
        }
      }
      if (misses_ >= config_.lost_misses_to_delete)
      {
        append_diagnostic(diagnostic, "lost 连续 miss 达到删除阈值，轨迹已删除");
        reset();
      }
    }
  }

  return make_result(timestamp_us, candidates, fused_beta_empty, associations, true, diagnostic);
}
