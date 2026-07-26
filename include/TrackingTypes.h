/**
 * @file TrackingTypes.h
 * @brief 在线跟踪共享数据类型与配置。
 *        Shared online-tracking data types and configuration.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "rs_driver/msg/pcl_point_cloud_msg.hpp"

using TrackingPointCloud = PointCloudT<PointXYZIRT>;
using TrackingPointCloudPtr = std::shared_ptr<TrackingPointCloud>;
using TrackingState9 = Eigen::Matrix<double, 9, 1>;
using TrackingCovariance9 = Eigen::Matrix<double, 9, 9>;

enum class TrackLifecycle
{
  INACTIVE,
  TENTATIVE,
  CONFIRMED,
  LOST
};

enum class TrackingControlSource
{
  HOLD,
  HOME,
  VISUAL_SERVO,
  PREDICTIVE_3D
};

struct Detection2D
{
  int class_id = 0;
  std::string class_name;
  double confidence = 0.0;
  double x1 = 0.0;
  double y1 = 0.0;
  double x2 = 0.0;
  double y2 = 0.0;

  double center_x() const
  {
    return 0.5 * (x1 + x2);
  }

  double center_y() const
  {
    return 0.5 * (y1 + y2);
  }
};

struct DetectionFrame
{
  uint64_t receive_sequence = 0;
  int schema_version = 1;
  uint64_t trigger_sequence = 0;
  int64_t lidar_end_timestamp_us = 0;
  int64_t lidar_callback_timestamp_us = 0;
  int64_t trigger_sent_timestamp_us = 0;
  int64_t trigger_received_timestamp_us = 0;
  int64_t trigger_command_timestamp_us = 0;
  int64_t source_timestamp_us = 0;
  int64_t host_callback_timestamp_us = 0;
  int64_t inference_start_timestamp_us = 0;
  int64_t inference_end_timestamp_us = 0;
  int64_t inference_done_timestamp_us = 0;
  int64_t receive_timestamp_us = 0;
  uint64_t camera_frame_id = 0;
  std::string status = "ok";
  std::string error;
  std::vector<Detection2D> detections;

  bool has_exact_lidar_timestamp() const
  {
    return schema_version >= 2 && lidar_end_timestamp_us > 0;
  }
};

struct CameraCalibration
{
  double fx = 2292.4112472233305;
  double fy = 2289.1280016360388;
  double cx = 1183.4111483043127;
  double cy = 1038.7331036789315;
  double k1 = -0.08272229239611316;
  double k2 = 0.13203832389665585;
  double p1 = 0.0003186630393428523;
  double p2 = -0.0032283162603578613;
  double k3 = -0.11291916041136751;
  int image_width = 2448;
  int image_height = 2048;
};

struct Candidate3D
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double visual_confidence = 0.0;
  int point_count = 0;
  bool from_motion = false;
  bool from_visual = false;
  Eigen::Vector2d projected_pixel = Eigen::Vector2d::Constant(-1.0);
};

struct MotionPrior
{
  bool valid = false;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
};

struct ModeTrackState
{
  TrackingState9 state = TrackingState9::Zero();
  TrackingCovariance9 covariance = TrackingCovariance9::Identity();
};

struct TrackResult
{
  bool state_valid = false;
  bool numerical_ok = true;
  int64_t frame_timestamp_us = 0;
  TrackLifecycle lifecycle = TrackLifecycle::INACTIVE;
  TrackingState9 state = TrackingState9::Zero();
  TrackingCovariance9 covariance = TrackingCovariance9::Identity();
  std::array<ModeTrackState, 2> modes;
  Eigen::Vector2d mode_probabilities = Eigen::Vector2d(0.9, 0.1);
  double beta_empty = 1.0;
  int hits = 0;
  int misses = 0;
  double lost_duration_s = 0.0;
  std::vector<Candidate3D> candidates;
  std::array<std::vector<double>, 2> mode_association_probabilities;
  std::string diagnostic;
};

struct TrackingCloudFrame
{
  TrackingPointCloudPtr cloud_world;
  int64_t lidar_start_timestamp_us = 0;
  int64_t lidar_end_timestamp_us = 0;
  // 保留原始同步键 / Preserve the original synchronization key.
  int64_t detection_match_timestamp_us = 0;
  int64_t deskew_start_timestamp_us = 0;
  int64_t deskew_done_timestamp_us = 0;
  int64_t deskew_processing_time_us = 0;
  int64_t tracking_submit_timestamp_us = 0;
  uint64_t input_point_count = 0;
  uint64_t output_point_count = 0;
  Eigen::Matrix4d T_W_V_end = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_W_G2_end = Eigen::Matrix4d::Identity();
  double gimbal_yaw_rad = 0.0;
  double gimbal_pitch_rad = 0.0;
  bool vehicle_pose_valid = false;
};

struct TrackingCommand
{
  bool valid = false;
  TrackingControlSource source = TrackingControlSource::HOLD;
  TrackLifecycle lifecycle = TrackLifecycle::INACTIVE;
  int64_t frame_timestamp_us = 0;
  int64_t generated_timestamp_us = 0;
  double stale_timeout_s = 0.30;
  double target_yaw_rad = 0.0;
  double target_pitch_rad = 0.0;
};

struct TrackingConfig
{
  bool enabled = true;
  bool static_vehicle = true;
  int udp_port = 11000;
  size_t detection_buffer_size = 64;
  size_t frame_queue_size = 2;
  int detection_wait_ms = 150;
  double detection_stale_s = 0.30;
  double minimum_detection_confidence = 0.5;
  double maximum_frame_gap_s = 0.5;
  double minimum_range_m = 1.0;
  double maximum_range_m = 70.0;
  double minimum_height_m = 0.0;
  double cluster_tolerance_m = 0.5;
  int cluster_min_points = 1;
  double motion_expansion_sigma = 1.0;
  double motion_gate_threshold = 11.345;
  double visual_baseline = 0.05;
  double visual_sigma = 0.5;
  double visual_association_bbox_scale_x = 1.0;
  double visual_association_bbox_scale_y = 1.0;
  double q_cv = 5.0;
  double q_ca = 3.0;
  double q_acceleration = 0.0005;
  double cv_acceleration_retention = 0.0;
  double transition_stay_probability = 0.95;
  double initial_cv_probability = 0.9;
  double initial_position_variance = 4.0;
  double initial_velocity_variance = 25.0;
  double initial_acceleration_variance = 25.0;
  double measurement_variance = 2.0;
  double association_epsilon = 0.01;
  double empty_weight = 0.001;
  double birth_confidence = 0.5;
  double hit_beta_empty_threshold = 0.6;
  int hits_to_confirm = 2;
  int tentative_misses_to_delete = 5;
  int confirmed_misses_to_lost = 10;
  int lost_misses_to_delete = 200;
  double lost_covariance_inflation = 1.05;
  double visual_servo_gain = 1.0;
  double servo_response_delay_s = 0.10;
  double maximum_prediction_horizon_s = 0.50;
  double tracking_stale_s = 0.30;
  double lost_hold_s = 1.0;
  int min_encoder_yaw = 1100;
  int max_encoder_yaw = 3000;
  int min_encoder_pitch = 1800;
  int max_encoder_pitch = 2800;
  double max_encoder_rate = 800.0;
  int home_encoder_yaw = 2048;
  int home_encoder_pitch = 2000;
  int servo_command_time_ms = 10;
  int home_move_time_ms = 500;
  int target_class_id = 0;
  double tentative_visual_association_ratio = 0.5;
};

inline const char* track_lifecycle_name(TrackLifecycle lifecycle)
{
  switch (lifecycle)
  {
  case TrackLifecycle::INACTIVE:
    return "inactive";
  case TrackLifecycle::TENTATIVE:
    return "tentative";
  case TrackLifecycle::CONFIRMED:
    return "confirmed";
  case TrackLifecycle::LOST:
    return "lost";
  }
  return "unknown";
}

inline const char* tracking_control_source_name(TrackingControlSource source)
{
  switch (source)
  {
  case TrackingControlSource::HOLD:
    return "hold";
  case TrackingControlSource::HOME:
    return "home";
  case TrackingControlSource::VISUAL_SERVO:
    return "visual_servo";
  case TrackingControlSource::PREDICTIVE_3D:
    return "predictive_3d";
  }
  return "unknown";
}
