/**
 * @file TargetTrackingPipeline.h
 * @brief 检测与点云同步跟踪流水线接口。
 *        Detection and point-cloud tracking pipeline.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "TrackingTypes.h"

/**
 * @brief 检测接收、点云跟踪与控制输出流水线。
 *        Detection, point-cloud tracking, and control-output pipeline.
 */
class TargetTrackingPipeline
{
public:
  enum class Mode
  {
    STOPPED,
    OBSERVE,
    ACTIVE
  };

  using CommandCallback = std::function<void(const TrackingCommand&)>;

  TargetTrackingPipeline();
  ~TargetTrackingPipeline();

  TargetTrackingPipeline(const TargetTrackingPipeline&) = delete;
  TargetTrackingPipeline& operator=(const TargetTrackingPipeline&) = delete;

  bool load_config(const std::string& tracker_config_path = "../config/tracker.json",
                   const std::string& camera_intrinsic_path = "../config/camera_intrinsic.json",
                   const std::string& extrinsic_config_path = "../config/extrinsic_calibration.json");

  bool start();

  void stop();

  void submit_frame(const TrackingCloudFrame& frame);

  void set_mode(Mode mode);
  Mode get_mode() const;
  TrackingConfig get_config() const;

  void set_command_callback(CommandCallback callback);

  bool start_saving(const std::string& directory = std::string());

  void stop_saving();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
