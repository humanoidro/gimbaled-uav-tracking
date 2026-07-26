/**
 * @file IMMPDAFTracker.h
 * @brief 单目标 IMM-PDAF 跟踪器接口。
 *        Single-target IMM-PDAF tracker interface.
 */

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "TrackingTypes.h"

/**
 * @brief 融合视觉与世界系点云的 IMM-PDAF 跟踪器。
 *        IMM-PDAF tracker for visual detections and world-frame point clouds.
 */
class IMMPDAFTracker
{
public:
  explicit IMMPDAFTracker(const TrackingConfig& config = TrackingConfig(),
                          const CameraCalibration& camera = CameraCalibration());

  void configure(const TrackingConfig& config, const CameraCalibration& camera);

  void reset();

  /// 处理一个同步帧 / Process one synchronized frame.
  TrackResult step(const TrackingPointCloud& cloud_world,
                   const DetectionFrame* detection,
                   const Eigen::Matrix4d& T_C_W,
                   int64_t timestamp_us,
                   double dt);

private:
  static constexpr int MODE_COUNT = 2;
  static constexpr int CV_MODE = 0;
  static constexpr int CA_MODE = 1;

  struct ProjectedPoint
  {
    Eigen::Vector3d world = Eigen::Vector3d::Zero();
    Eigen::Vector2d pixel = Eigen::Vector2d::Constant(-1.0);
    bool projection_valid = false;
    bool from_motion = false;
    bool from_visual = false;
  };

  struct ModeUpdate
  {
    ModeTrackState posterior;
    std::vector<double> association_probabilities;
    double log_evidence = 0.0;
  };

  bool validate_configuration(std::string& diagnostic) const;
  bool project_world_point(const Eigen::Vector3d& point_world,
                           const Eigen::Matrix4d& T_C_W,
                           Eigen::Vector2d& pixel,
                           double* range_m = nullptr) const;
  std::vector<Candidate3D> generate_candidates(const TrackingPointCloud& cloud_world,
                                                const DetectionFrame* detection,
                                                const Eigen::Matrix4d& T_C_W,
                                                const MotionPrior& motion_prior,
                                                std::string& diagnostic) const;

  void initialize_track(const Eigen::Vector3d& position);
  bool predict_modes(double dt,
                     std::array<ModeTrackState, MODE_COUNT>& predicted_modes,
                     Eigen::Vector2d& predicted_probabilities,
                     std::string& diagnostic) const;
  bool update_mode(const ModeTrackState& predicted,
                   const std::vector<Candidate3D>& candidates,
                   ModeUpdate& update,
                   std::string& diagnostic) const;
  bool fuse_modes(const std::array<ModeTrackState, MODE_COUNT>& modes,
                  const Eigen::Vector2d& probabilities,
                  TrackingState9& state,
                  TrackingCovariance9& covariance) const;
  bool stabilize_covariance(TrackingCovariance9& covariance) const;
  bool all_state_finite(const std::array<ModeTrackState, MODE_COUNT>& modes,
                        const Eigen::Vector2d& probabilities) const;

  TrackResult make_result(int64_t timestamp_us,
                          const std::vector<Candidate3D>& candidates,
                          double beta_empty,
                          const std::array<std::vector<double>, MODE_COUNT>& associations,
                          bool numerical_ok,
                          const std::string& diagnostic) const;

  TrackingConfig config_;
  CameraCalibration camera_;
  bool configuration_valid_ = false;
  std::string configuration_diagnostic_;

  bool initialized_ = false;
  TrackLifecycle lifecycle_ = TrackLifecycle::INACTIVE;
  std::array<ModeTrackState, MODE_COUNT> modes_;
  Eigen::Vector2d mode_probabilities_ = Eigen::Vector2d(0.9, 0.1);
  int hits_ = 0;
  int misses_ = 0;
  double lost_duration_s_ = 0.0;
};
