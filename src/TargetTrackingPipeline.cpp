/**
 * @file TargetTrackingPipeline.cpp
 * @brief 检测与点云同步跟踪流水线实现。
 *        Detection and point-cloud tracking pipeline.
 */

#include "TargetTrackingPipeline.h"

#include "AppConfig.h"
#include "IMMPDAFTracker.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>
#include <utility>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

namespace
{
using Json = nlohmann::json;

int64_t unix_time_us()
{
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

int64_t elapsed_time_us(int64_t end_timestamp_us, int64_t start_timestamp_us)
{
  if (end_timestamp_us <= 0 || start_timestamp_us <= 0 || end_timestamp_us < start_timestamp_us)
  {
    return -1;
  }
  return end_timestamp_us - start_timestamp_us;
}

template <typename T>
T nested_value(const Json& root, const char* group, const char* key, const T& fallback)
{
  if (!root.contains(group) || !root[group].is_object())
  {
    return fallback;
  }
  return root[group].value(key, fallback);
}

bool read_json_file(const std::string& path, Json& output)
{
  std::vector<std::string> candidates = {path};
  if (path.rfind("../config/", 0) == 0)
  {
    candidates.push_back(path.substr(3));
  }
  else if (path.rfind("config/", 0) == 0)
  {
    candidates.push_back("../" + path);
  }

  for (const auto& candidate : candidates)
  {
    std::ifstream stream(candidate);
    if (!stream.good())
    {
      continue;
    }
    try
    {
      output = Json::parse(stream);
      return true;
    }
    catch (const std::exception& e)
    {
      std::cerr << "TargetTrackingPipeline: 解析配置失败 " << candidate << ": " << e.what() << std::endl;
      return false;
    }
  }

  std::cerr << "TargetTrackingPipeline: 无法打开配置文件 " << path << std::endl;
  return false;
}

bool finite_transform(const Eigen::Matrix4d& transform)
{
  if (!transform.allFinite())
  {
    return false;
  }
  if ((transform.row(3) - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm() > 1e-6)
  {
    return false;
  }
  const Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
  return std::abs(rotation.determinant()) > 1e-6;
}

bool load_tracker_config(const Json& json, TrackingConfig& config)
{
  try
  {
    config.enabled = json.value("enabled", config.enabled);
    config.static_vehicle = nested_value(json, "platform", "static_vehicle", config.static_vehicle);

    config.udp_port = nested_value(json, "detection", "udp_port", config.udp_port);
    config.detection_buffer_size = nested_value(
        json, "detection", "buffer_size", config.detection_buffer_size);
    config.detection_wait_ms = nested_value(json, "detection", "wait_ms", config.detection_wait_ms);
    config.detection_stale_s = nested_value(
        json, "detection", "stale_s", config.detection_stale_s);
    config.minimum_detection_confidence = nested_value(
        json, "detection", "minimum_confidence", config.minimum_detection_confidence);
    config.target_class_id = nested_value(
        json, "detection", "target_class_id", config.target_class_id);

    config.frame_queue_size = nested_value(json, "runtime", "frame_queue_size", config.frame_queue_size);
    config.maximum_frame_gap_s = nested_value(
        json, "runtime", "maximum_frame_gap_s", config.maximum_frame_gap_s);

    config.minimum_range_m = nested_value(json, "candidate", "minimum_range_m", config.minimum_range_m);
    config.maximum_range_m = nested_value(json, "candidate", "maximum_range_m", config.maximum_range_m);
    config.minimum_height_m = nested_value(
        json, "candidate", "minimum_height_m", config.minimum_height_m);
    config.cluster_tolerance_m = nested_value(
        json, "candidate", "cluster_tolerance_m", config.cluster_tolerance_m);
    config.cluster_min_points = nested_value(
        json, "candidate", "cluster_min_points", config.cluster_min_points);
    config.motion_expansion_sigma = nested_value(
        json, "candidate", "motion_expansion_sigma", config.motion_expansion_sigma);
    config.motion_gate_threshold = nested_value(
        json, "candidate", "motion_gate_threshold", config.motion_gate_threshold);
    config.visual_baseline = nested_value(json, "candidate", "visual_baseline", config.visual_baseline);
    config.visual_sigma = nested_value(json, "candidate", "visual_sigma", config.visual_sigma);
    config.visual_association_bbox_scale_x = nested_value(
        json, "candidate", "visual_association_bbox_scale_x",
        config.visual_association_bbox_scale_x);
    config.visual_association_bbox_scale_y = nested_value(
        json, "candidate", "visual_association_bbox_scale_y",
        config.visual_association_bbox_scale_y);

    config.q_cv = nested_value(json, "imm", "q_cv", config.q_cv);
    config.q_ca = nested_value(json, "imm", "q_ca", config.q_ca);
    config.q_acceleration = nested_value(json, "imm", "q_acceleration", config.q_acceleration);
    config.cv_acceleration_retention = nested_value(
        json, "imm", "cv_acceleration_retention", config.cv_acceleration_retention);
    config.transition_stay_probability = nested_value(
        json, "imm", "transition_stay_probability", config.transition_stay_probability);
    config.initial_cv_probability = nested_value(
        json, "imm", "initial_cv_probability", config.initial_cv_probability);
    config.initial_position_variance = nested_value(
        json, "imm", "initial_position_variance", config.initial_position_variance);
    config.initial_velocity_variance = nested_value(
        json, "imm", "initial_velocity_variance", config.initial_velocity_variance);
    config.initial_acceleration_variance = nested_value(
        json, "imm", "initial_acceleration_variance", config.initial_acceleration_variance);

    config.measurement_variance = nested_value(
        json, "pdaf", "measurement_variance", config.measurement_variance);
    config.association_epsilon = nested_value(
        json, "pdaf", "association_epsilon", config.association_epsilon);
    config.empty_weight = nested_value(json, "pdaf", "empty_weight", config.empty_weight);

    config.birth_confidence = nested_value(
        json, "lifecycle", "birth_confidence", config.birth_confidence);
    config.hit_beta_empty_threshold = nested_value(
        json, "lifecycle", "hit_beta_empty_threshold", config.hit_beta_empty_threshold);
    config.tentative_visual_association_ratio = nested_value(
        json, "lifecycle", "tentative_visual_association_ratio",
        config.tentative_visual_association_ratio);
    config.hits_to_confirm = nested_value(
        json, "lifecycle", "hits_to_confirm", config.hits_to_confirm);
    config.tentative_misses_to_delete = nested_value(
        json, "lifecycle", "tentative_misses_to_delete", config.tentative_misses_to_delete);
    config.confirmed_misses_to_lost = nested_value(
        json, "lifecycle", "confirmed_misses_to_lost", config.confirmed_misses_to_lost);
    config.lost_misses_to_delete = nested_value(
        json, "lifecycle", "lost_misses_to_delete", config.lost_misses_to_delete);
    config.lost_covariance_inflation = nested_value(
        json, "lifecycle", "lost_covariance_inflation", config.lost_covariance_inflation);

    config.visual_servo_gain = nested_value(
        json, "control", "visual_servo_gain", config.visual_servo_gain);
    config.servo_response_delay_s = nested_value(
        json, "control", "servo_response_delay_s", config.servo_response_delay_s);
    config.maximum_prediction_horizon_s = nested_value(
        json, "control", "maximum_prediction_horizon_s", config.maximum_prediction_horizon_s);
    config.tracking_stale_s = nested_value(
        json, "control", "tracking_stale_s", config.tracking_stale_s);
    config.lost_hold_s = nested_value(json, "control", "lost_hold_s", config.lost_hold_s);
    config.min_encoder_yaw = nested_value(
        json, "control", "min_encoder_yaw", config.min_encoder_yaw);
    config.max_encoder_yaw = nested_value(
        json, "control", "max_encoder_yaw", config.max_encoder_yaw);
    config.min_encoder_pitch = nested_value(
        json, "control", "min_encoder_pitch", config.min_encoder_pitch);
    config.max_encoder_pitch = nested_value(
        json, "control", "max_encoder_pitch", config.max_encoder_pitch);
    config.max_encoder_rate = nested_value(
        json, "control", "max_encoder_rate", config.max_encoder_rate);
    config.home_encoder_yaw = nested_value(
        json, "control", "home_encoder_yaw", config.home_encoder_yaw);
    config.home_encoder_pitch = nested_value(
        json, "control", "home_encoder_pitch", config.home_encoder_pitch);
    config.servo_command_time_ms = nested_value(
        json, "control", "servo_command_time_ms", config.servo_command_time_ms);
    config.home_move_time_ms = nested_value(
        json, "control", "home_move_time_ms", config.home_move_time_ms);
  }
  catch (const std::exception& e)
  {
    std::cerr << "TargetTrackingPipeline: tracker.json 参数类型错误: " << e.what() << std::endl;
    return false;
  }

  const bool probabilities_valid = config.transition_stay_probability > 0.0
      && config.transition_stay_probability < 1.0 && config.initial_cv_probability > 0.0
      && config.initial_cv_probability < 1.0 && config.hit_beta_empty_threshold > 0.0
      && config.hit_beta_empty_threshold < 1.0
      && std::isfinite(config.tentative_visual_association_ratio)
      && config.tentative_visual_association_ratio >= 0.0
      && config.tentative_visual_association_ratio <= 1.0;
  const bool dimensions_valid = config.udp_port > 0 && config.udp_port <= 65535
      && config.detection_buffer_size > 0 && config.frame_queue_size > 0
      && config.detection_wait_ms >= 0 && config.cluster_min_points > 0
      && config.hits_to_confirm > 0 && config.tentative_misses_to_delete > 0
      && config.confirmed_misses_to_lost > 0 && config.lost_misses_to_delete > 0
      && config.min_encoder_yaw >= 0 && config.max_encoder_yaw <= 65535
      && config.min_encoder_pitch >= 0 && config.max_encoder_pitch <= 65535
      && config.min_encoder_yaw < config.max_encoder_yaw
      && config.min_encoder_pitch < config.max_encoder_pitch
      && config.home_encoder_yaw >= config.min_encoder_yaw
      && config.home_encoder_yaw <= config.max_encoder_yaw
      && config.home_encoder_pitch >= config.min_encoder_pitch
      && config.home_encoder_pitch <= config.max_encoder_pitch
      && config.servo_command_time_ms > 0 && config.servo_command_time_ms <= 65535
      && config.home_move_time_ms > 0 && config.home_move_time_ms <= 65535;
  const bool positive_valid = config.maximum_frame_gap_s > 0.0 && config.minimum_range_m >= 0.0
      && config.maximum_range_m > config.minimum_range_m
      && std::isfinite(config.minimum_height_m) && config.minimum_height_m >= 0.0
      && config.cluster_tolerance_m > 0.0
      && config.motion_expansion_sigma >= 0.0 && config.motion_gate_threshold > 0.0
      && config.visual_sigma > 0.0
      && std::isfinite(config.visual_association_bbox_scale_x)
      && config.visual_association_bbox_scale_x >= 1.0
      && std::isfinite(config.visual_association_bbox_scale_y)
      && config.visual_association_bbox_scale_y >= 1.0
      && config.q_cv >= 0.0 && config.q_ca >= 0.0
      && config.q_acceleration >= 0.0 && config.measurement_variance > 0.0
      && config.association_epsilon >= 0.0 && config.empty_weight > 0.0
      && config.lost_covariance_inflation >= 1.0 && config.visual_servo_gain > 0.0
      && config.servo_response_delay_s >= 0.0 && config.maximum_prediction_horizon_s > 0.0
      && config.tracking_stale_s > 0.0 && config.detection_stale_s > 0.0
      && config.minimum_detection_confidence >= 0.0
      && config.minimum_detection_confidence <= 1.0 && config.lost_hold_s >= 0.0
      && config.max_encoder_rate > 0.0;

  if (!probabilities_valid || !dimensions_valid || !positive_valid)
  {
    std::cerr << "TargetTrackingPipeline: tracker.json 含越界参数" << std::endl;
    return false;
  }
  return true;
}

bool load_camera_config(const Json& json, CameraCalibration& camera)
{
  try
  {
    camera.fx = json.value("fx", camera.fx);
    camera.fy = json.value("fy", camera.fy);
    camera.cx = json.value("cx", camera.cx);
    camera.cy = json.value("cy", camera.cy);
    camera.k1 = json.value("k1", camera.k1);
    camera.k2 = json.value("k2", camera.k2);
    camera.p1 = json.value("p1", camera.p1);
    camera.p2 = json.value("p2", camera.p2);
    camera.k3 = json.value("k3", camera.k3);
    camera.image_width = json.value("image_width", camera.image_width);
    camera.image_height = json.value("image_height", camera.image_height);
  }
  catch (const std::exception& e)
  {
    std::cerr << "TargetTrackingPipeline: 相机内参类型错误: " << e.what() << std::endl;
    return false;
  }

  const std::array<double, 9> values = {
      camera.fx, camera.fy, camera.cx, camera.cy, camera.k1,
      camera.k2, camera.p1, camera.p2, camera.k3};
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); })
      && camera.fx > 0.0 && camera.fy > 0.0 && camera.image_width > 0 && camera.image_height > 0;
}

bool load_extrinsic_config(const Json& json, Eigen::Matrix4d& T_G2_C)
{
  if (!json.contains("T_G2_C") || !json["T_G2_C"].is_object())
  {
    std::cerr << "TargetTrackingPipeline: 外参文件缺少 T_G2_C" << std::endl;
    return false;
  }

  try
  {
    const Json& node = json["T_G2_C"];
    const Json& rotation = node.at("rotation");
    const Json& translation = node.at("translation");
    if (!rotation.is_array() || rotation.size() != 3 || !translation.is_array()
        || translation.size() != 3)
    {
      return false;
    }

    T_G2_C.setIdentity();
    for (int row = 0; row < 3; ++row)
    {
      if (!rotation[row].is_array() || rotation[row].size() != 3)
      {
        return false;
      }
      for (int column = 0; column < 3; ++column)
      {
        T_G2_C(row, column) = rotation[row][column].get<double>();
      }
      T_G2_C(row, 3) = translation[row].get<double>();
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "TargetTrackingPipeline: T_G2_C 解析失败: " << e.what() << std::endl;
    return false;
  }

  if (!finite_transform(T_G2_C))
  {
    return false;
  }
  const Eigen::Matrix3d rotation = T_G2_C.block<3, 3>(0, 0);
  return (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() < 1e-3
      && std::abs(rotation.determinant() - 1.0) < 1e-3;
}

bool number_at(const Json& object, const char* key, double& value)
{
  if (!object.contains(key) || !object[key].is_number())
  {
    return false;
  }
  value = object[key].get<double>();
  return std::isfinite(value);
}

template <typename T>
void read_integer_if_present(const Json& object, const char* key, T& value)
{
  if (object.contains(key) && object[key].is_number_integer())
  {
    value = object[key].get<T>();
  }
}

bool parse_detection_message(const std::string& message, DetectionFrame& frame, std::string& error)
{
  try
  {
    const Json root = Json::parse(message);
    if (!root.is_object())
    {
      error = "顶层不是对象";
      return false;
    }

    frame.schema_version = root.value("schema_version", 1);
    if (frame.schema_version < 1 || frame.schema_version > 2)
    {
      error = "不支持的 schema_version";
      return false;
    }

    read_integer_if_present(root, "trigger_sequence", frame.trigger_sequence);
    read_integer_if_present(root, "lidar_end_timestamp_us", frame.lidar_end_timestamp_us);
    read_integer_if_present(root, "lidar_callback_timestamp_us", frame.lidar_callback_timestamp_us);
    read_integer_if_present(root, "trigger_sent_timestamp_us", frame.trigger_sent_timestamp_us);
    read_integer_if_present(root, "trigger_received_timestamp_us", frame.trigger_received_timestamp_us);
    read_integer_if_present(root, "trigger_command_timestamp_us", frame.trigger_command_timestamp_us);
    read_integer_if_present(root, "source_timestamp_us", frame.source_timestamp_us);
    read_integer_if_present(root, "host_callback_timestamp_us", frame.host_callback_timestamp_us);
    read_integer_if_present(root, "inference_start_timestamp_us", frame.inference_start_timestamp_us);
    read_integer_if_present(root, "inference_end_timestamp_us", frame.inference_end_timestamp_us);
    read_integer_if_present(root, "inference_done_timestamp_us", frame.inference_done_timestamp_us);
    read_integer_if_present(root, "camera_frame_id", frame.camera_frame_id);
    if (root.contains("status") && root["status"].is_string())
    {
      frame.status = root["status"].get<std::string>();
    }
    if (root.contains("error") && root["error"].is_string())
    {
      frame.error = root["error"].get<std::string>();
    }

    int64_t legacy_timestamp_us = 0;
    read_integer_if_present(root, "ts", legacy_timestamp_us);
    if (frame.source_timestamp_us <= 0)
    {
      frame.source_timestamp_us = legacy_timestamp_us;
    }

    if (frame.schema_version >= 2 && frame.lidar_end_timestamp_us <= 0)
    {
      error = "v2 消息缺少 lidar_end_timestamp_us";
      return false;
    }

    const Json* detections = nullptr;
    if (root.contains("d"))
    {
      detections = &root["d"];
    }
    else if (root.contains("detections"))
    {
      detections = &root["detections"];
    }
    if (!detections || !detections->is_array())
    {
      error = "缺少检测数组";
      return false;
    }

    for (const Json& item : *detections)
    {
      if (!item.is_object())
      {
        continue;
      }
      Detection2D detection;
      if (item.contains("c") && item["c"].is_number_integer())
      {
        detection.class_id = item["c"].get<int>();
      }
      else if (item.contains("class_id") && item["class_id"].is_number_integer())
      {
        detection.class_id = item["class_id"].get<int>();
      }
      if (item.contains("cn") && item["cn"].is_string())
      {
        detection.class_name = item["cn"].get<std::string>();
      }
      else if (item.contains("class_name") && item["class_name"].is_string())
      {
        detection.class_name = item["class_name"].get<std::string>();
      }

      bool confidence_valid = number_at(item, "conf", detection.confidence);
      if (!confidence_valid)
      {
        confidence_valid = number_at(item, "confidence", detection.confidence);
      }

      bool bbox_valid = false;
      if (item.contains("bbox") && item["bbox"].is_array() && item["bbox"].size() >= 4)
      {
        detection.x1 = item["bbox"][0].get<double>();
        detection.y1 = item["bbox"][1].get<double>();
        detection.x2 = item["bbox"][2].get<double>();
        detection.y2 = item["bbox"][3].get<double>();
        bbox_valid = std::isfinite(detection.x1) && std::isfinite(detection.y1)
            && std::isfinite(detection.x2) && std::isfinite(detection.y2);
      }
      else
      {
        bbox_valid = number_at(item, "x1", detection.x1) && number_at(item, "y1", detection.y1)
            && number_at(item, "x2", detection.x2) && number_at(item, "y2", detection.y2);
      }

      if (confidence_valid && bbox_valid && detection.confidence >= 0.0
          && detection.confidence <= 1.0 && detection.x2 >= detection.x1 && detection.y2 >= detection.y1)
      {
        frame.detections.push_back(std::move(detection));
      }
    }
    return true;
  }
  catch (const std::exception& e)
  {
    error = e.what();
    return false;
  }
}

std::string csv_text(const std::string& input)
{
  std::string result = "\"";
  for (char ch : input)
  {
    if (ch == '"')
    {
      result += "\"\"";
    }
    else if (ch == '\n' || ch == '\r')
    {
      result += ' ';
    }
    else
    {
      result += ch;
    }
  }
  result += '"';
  return result;
}

std::string time_suffix()
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                                now.time_since_epoch())
                                .count()
      % 1000;
  std::tm local{};
  localtime_r(&time, &local);
  std::ostringstream stream;
  stream << std::setfill('0') << std::setw(4) << local.tm_year + 1900 << std::setw(2)
         << local.tm_mon + 1 << std::setw(2) << local.tm_mday << '_' << std::setw(2)
         << local.tm_hour << std::setw(2) << local.tm_min << std::setw(2) << local.tm_sec
         << '_' << std::setw(3) << milliseconds;
  return stream.str();
}

void fix_permissions(const std::string& path)
{
  const char* uid_text = std::getenv("SUDO_UID");
  const char* gid_text = std::getenv("SUDO_GID");
  if (!uid_text || !gid_text)
  {
    return;
  }
  try
  {
    const uid_t uid = static_cast<uid_t>(std::stoul(uid_text));
    const gid_t gid = static_cast<gid_t>(std::stoul(gid_text));
    if (chown(path.c_str(), uid, gid) != 0)
    {
      // 权限回写失败不影响保存 / Permission fallback does not affect saving.
    }
    struct stat status{};
    if (stat(path.c_str(), &status) == 0)
    {
      chmod(path.c_str(), S_ISDIR(status.st_mode) ? 0775 : 0664);
    }
  }
  catch (const std::exception&)
  {
  }
}

double candidate_probability(const std::vector<double>& probabilities, size_t candidate_index,
                             size_t candidate_count)
{
  if (probabilities.size() == candidate_count + 1)
  {
    return probabilities[candidate_index + 1];
  }
  if (probabilities.size() == candidate_count)
  {
    return probabilities[candidate_index];
  }
  return 0.0;
}
}  // namespace

struct TargetTrackingPipeline::Impl
{
  TrackingConfig config;
  CameraCalibration camera;
  Eigen::Matrix4d T_G2_C = Eigen::Matrix4d::Identity();
  IMMPDAFTracker tracker;

  std::atomic<bool> configured{false};
  std::atomic<bool> running{false};
  std::atomic<Mode> mode{Mode::STOPPED};
  std::atomic<int> udp_socket{-1};
  std::thread receiver_thread;
  std::thread tracking_thread;

  mutable std::mutex detection_mutex;
  std::condition_variable detection_cv;
  std::deque<DetectionFrame> detection_buffer;
  uint64_t next_detection_sequence = 1;

  std::mutex frame_mutex;
  std::condition_variable frame_cv;
  std::deque<TrackingCloudFrame> frame_queue;
  std::atomic<uint64_t> dropped_frames{0};

  std::mutex tracker_mutex;
  int64_t previous_frame_timestamp_us = 0;

  std::mutex callback_mutex;
  CommandCallback command_callback;

  std::atomic<bool> saving{false};
  std::mutex saving_mutex;
  std::ofstream detection_stream;
  std::ofstream tracking_stream;
  std::ofstream candidate_stream;
  std::ofstream detection_timing_stream;
  std::ofstream tracking_timing_stream;
  std::string detection_filename;
  std::string tracking_filename;
  std::string candidate_filename;
  std::string detection_timing_filename;
  std::string tracking_timing_filename;

  void reset_tracking_state()
  {
    {
      std::lock_guard<std::mutex> lock(tracker_mutex);
      tracker.reset();
      previous_frame_timestamp_us = 0;
    }
    {
      std::lock_guard<std::mutex> lock(frame_mutex);
      frame_queue.clear();
    }
  }

  bool find_exact_detection_locked(int64_t timestamp_us, DetectionFrame& detection) const
  {
    for (auto iterator = detection_buffer.rbegin(); iterator != detection_buffer.rend(); ++iterator)
    {
      if (iterator->has_exact_lidar_timestamp()
          && iterator->lidar_end_timestamp_us == timestamp_us)
      {
        detection = *iterator;
        return true;
      }
    }
    return false;
  }

  bool wait_for_exact_detection(int64_t timestamp_us, DetectionFrame& detection)
  {
    std::unique_lock<std::mutex> lock(detection_mutex);
    const auto ready = [this, timestamp_us, &detection]()
    {
      return !running.load() || mode.load() == Mode::STOPPED
          || find_exact_detection_locked(timestamp_us, detection);
    };
    if (!ready())
    {
      detection_cv.wait_for(lock, std::chrono::milliseconds(config.detection_wait_ms), ready);
    }
    return find_exact_detection_locked(timestamp_us, detection);
  }

  void log_detection(const DetectionFrame& frame)
  {
    if (!saving.load())
    {
      return;
    }
    std::lock_guard<std::mutex> lock(saving_mutex);
    if (!saving.load() || !detection_stream.good())
    {
      return;
    }

    auto write_prefix = [this, &frame](int index)
    {
      detection_stream << frame.receive_sequence << ',' << frame.schema_version << ','
                       << frame.trigger_sequence << ',' << frame.lidar_end_timestamp_us << ','
                       << frame.source_timestamp_us << ',' << frame.host_callback_timestamp_us << ','
                       << frame.inference_done_timestamp_us << ',' << frame.receive_timestamp_us << ','
                       << frame.camera_frame_id << ',' << csv_text(frame.status) << ','
                       << csv_text(frame.error) << ',' << frame.detections.size() << ',' << index;
    };
    if (frame.detections.empty())
    {
      write_prefix(-1);
      detection_stream << ",-1,\"\",0,0,0,0,0\n";
    }
    else
    {
      for (size_t index = 0; index < frame.detections.size(); ++index)
      {
        const Detection2D& item = frame.detections[index];
        write_prefix(static_cast<int>(index));
        detection_stream << ',' << item.class_id << ',' << csv_text(item.class_name) << ','
                         << item.confidence << ',' << item.x1 << ',' << item.y1 << ',' << item.x2
                         << ',' << item.y2 << '\n';
      }
    }

    if (!detection_timing_stream.good())
    {
      return;
    }
    detection_timing_stream
        << frame.receive_sequence << ',' << frame.schema_version << ',' << frame.trigger_sequence << ','
        << frame.lidar_end_timestamp_us << ',' << csv_text(frame.status) << ',' << csv_text(frame.error)
        << ',' << frame.camera_frame_id << ',' << frame.lidar_callback_timestamp_us << ','
        << frame.trigger_sent_timestamp_us << ',' << frame.trigger_received_timestamp_us << ','
        << frame.trigger_command_timestamp_us << ',' << frame.host_callback_timestamp_us << ','
        << frame.inference_start_timestamp_us << ',' << frame.inference_end_timestamp_us << ','
        << frame.inference_done_timestamp_us << ',' << frame.receive_timestamp_us << ','
        << elapsed_time_us(frame.lidar_callback_timestamp_us, frame.lidar_end_timestamp_us) << ','
        << elapsed_time_us(frame.trigger_sent_timestamp_us, frame.lidar_callback_timestamp_us) << ','
        << elapsed_time_us(frame.trigger_received_timestamp_us, frame.trigger_sent_timestamp_us) << ','
        << elapsed_time_us(frame.host_callback_timestamp_us, frame.trigger_command_timestamp_us) << ','
        << elapsed_time_us(frame.inference_start_timestamp_us, frame.host_callback_timestamp_us) << ','
        << elapsed_time_us(frame.inference_end_timestamp_us, frame.inference_start_timestamp_us) << ','
        << elapsed_time_us(frame.inference_done_timestamp_us, frame.inference_end_timestamp_us) << ','
        << elapsed_time_us(frame.receive_timestamp_us, frame.inference_done_timestamp_us) << ','
        << elapsed_time_us(frame.inference_done_timestamp_us, frame.trigger_command_timestamp_us) << '\n';
  }

  void receive_loop()
  {
    std::array<char, 65536> buffer{};
    while (running.load())
    {
      const int socket_fd = udp_socket.load();
      if (socket_fd < 0)
      {
        break;
      }

      fd_set read_set;
      FD_ZERO(&read_set);
      FD_SET(socket_fd, &read_set);
      timeval timeout{};
      timeout.tv_usec = 100000;
      const int selected = select(socket_fd + 1, &read_set, nullptr, nullptr, &timeout);
      if (selected < 0)
      {
        if (running.load())
        {
          std::cerr << "TargetTrackingPipeline: YOLO UDP select 失败" << std::endl;
        }
        break;
      }
      if (selected == 0)
      {
        continue;
      }

      const ssize_t received = recvfrom(socket_fd, buffer.data(), buffer.size() - 1, 0, nullptr, nullptr);
      if (received <= 0)
      {
        if (!running.load())
        {
          break;
        }
        continue;
      }

      DetectionFrame frame;
      frame.receive_timestamp_us = unix_time_us();
      std::string error;
      if (!parse_detection_message(std::string(buffer.data(), static_cast<size_t>(received)), frame, error))
      {
        std::cerr << "TargetTrackingPipeline: 丢弃无效 YOLO 消息: " << error << std::endl;
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(detection_mutex);
        frame.receive_sequence = next_detection_sequence++;
        detection_buffer.push_back(frame);
        while (detection_buffer.size() > config.detection_buffer_size)
        {
          detection_buffer.pop_front();
        }
      }
      detection_cv.notify_all();
      log_detection(frame);
    }

    const int socket_fd = udp_socket.exchange(-1);
    if (socket_fd >= 0)
    {
      close(socket_fd);
    }
  }

  const Detection2D* select_target_detection(const DetectionFrame* frame) const
  {
    if (!frame || frame->status != "ok")
    {
      return nullptr;
    }
    const Detection2D* selected = nullptr;
    for (const Detection2D& detection : frame->detections)
    {
      if (detection.class_id != config.target_class_id)
      {
        continue;
      }
      if (!selected || detection.confidence > selected->confidence)
      {
        selected = &detection;
      }
    }
    return selected;
  }

  TrackingCommand build_command(const TrackingCloudFrame& frame, const DetectionFrame* detection,
                                const TrackResult& result) const
  {
    TrackingCommand command;
    command.lifecycle = result.lifecycle;
    command.frame_timestamp_us = frame.lidar_end_timestamp_us;
    command.generated_timestamp_us = unix_time_us();
    command.stale_timeout_s = config.tracking_stale_s;

    if (mode.load() != Mode::ACTIVE)
    {
      return command;
    }

    const double frame_age_s = static_cast<double>(command.generated_timestamp_us
                                                    - frame.lidar_end_timestamp_us)
        * 1e-6;
    if (frame_age_s > config.tracking_stale_s)
    {
      return command;
    }

    if (result.lifecycle == TrackLifecycle::INACTIVE ||
        result.lifecycle == TrackLifecycle::TENTATIVE)
    {
      const Detection2D* selected = select_target_detection(detection);
      if (!selected)
      {
        if (result.lifecycle == TrackLifecycle::INACTIVE)
        {
          command.valid = true;
          command.source = TrackingControlSource::HOME;
        }
        return command;
      }

      const double normalized_x = (selected->center_x() - camera.cx) / camera.fx;
      const double normalized_y = (selected->center_y() - camera.cy) / camera.fy;
      command.target_yaw_rad = frame.gimbal_yaw_rad
          - config.visual_servo_gain * std::atan(normalized_x);
      command.target_pitch_rad = frame.gimbal_pitch_rad
          + config.visual_servo_gain * std::atan(normalized_y);
      command.valid = std::isfinite(command.target_yaw_rad) && std::isfinite(command.target_pitch_rad);
      command.source = command.valid ? TrackingControlSource::VISUAL_SERVO
                                     : TrackingControlSource::HOLD;
      return command;
    }

    if (!result.state_valid || !result.numerical_ok || !result.state.allFinite())
    {
      return command;
    }
    if (result.lifecycle == TrackLifecycle::LOST && result.lost_duration_s > config.lost_hold_s)
    {
      command.valid = true;
      command.source = TrackingControlSource::HOME;
      return command;
    }
    if (!config.static_vehicle && !frame.vehicle_pose_valid)
    {
      return command;
    }
    if (!finite_transform(frame.T_W_V_end) || !finite_transform(frame.T_W_G2_end))
    {
      return command;
    }

    const double latency_s = std::max(0.0, frame_age_s);
    const double horizon = std::clamp(latency_s + config.servo_response_delay_s, 0.0,
                                      config.maximum_prediction_horizon_s);
    Eigen::Vector3d predicted_world = Eigen::Vector3d::Zero();
    double probability_sum = 0.0;
    for (size_t index = 0; index < result.modes.size(); ++index)
    {
      if (!result.modes[index].state.allFinite())
      {
        continue;
      }
      const TrackingState9& state = result.modes[index].state;
      Eigen::Vector3d predicted = state.segment<3>(0) + horizon * state.segment<3>(3);
      if (index == 1)
      {
        predicted += 0.5 * horizon * horizon * state.segment<3>(6);
      }
      const double probability = std::max(0.0, result.mode_probabilities(static_cast<int>(index)));
      predicted_world += probability * predicted;
      probability_sum += probability;
    }
    if (probability_sum <= 1e-12)
    {
      return command;
    }
    predicted_world /= probability_sum;

    const Eigen::Matrix4d T_V_W = frame.T_W_V_end.inverse();
    const Eigen::Vector3d target_vehicle =
        (T_V_W * predicted_world.homogeneous()).head<3>();
    const Eigen::Matrix4d T_V_C = T_V_W * frame.T_W_G2_end * T_G2_C;
    const Eigen::Vector3d camera_vehicle = T_V_C.block<3, 1>(0, 3);
    const Eigen::Vector3d ray = target_vehicle - camera_vehicle;
    const double horizontal = std::hypot(ray.x(), ray.y());
    if (!ray.allFinite() || ray.norm() < 1e-6 || horizontal < 1e-9)
    {
      return command;
    }

    command.target_yaw_rad = std::atan2(ray.y(), ray.x());
    command.target_pitch_rad = std::atan2(-ray.z(), horizontal);
    command.valid = std::isfinite(command.target_yaw_rad) && std::isfinite(command.target_pitch_rad);
    command.source = command.valid ? TrackingControlSource::PREDICTIVE_3D
                                   : TrackingControlSource::HOLD;
    return command;
  }

  void log_tracking(const TrackingCloudFrame& frame, const DetectionFrame* detection,
                    bool detection_matched, const TrackResult& result,
                    const TrackingCommand& command, int64_t tracking_dequeue_timestamp_us,
                    int64_t detection_wait_time_us, int64_t processing_time_us)
  {
    if (!saving.load())
    {
      return;
    }
    std::lock_guard<std::mutex> lock(saving_mutex);
    if (!saving.load() || !tracking_stream.good() || !candidate_stream.good())
    {
      return;
    }
    const int64_t completed_timestamp_us = unix_time_us();

    const int64_t lidar_callback_timestamp_us = detection
        ? detection->lidar_callback_timestamp_us : 0;
    const int64_t trigger_sent_timestamp_us = detection
        ? detection->trigger_sent_timestamp_us : 0;
    const int64_t trigger_received_timestamp_us = detection
        ? detection->trigger_received_timestamp_us : 0;
    const int64_t trigger_command_timestamp_us = detection
        ? detection->trigger_command_timestamp_us : 0;
    const int64_t camera_callback_timestamp_us = detection
        ? detection->host_callback_timestamp_us : 0;
    const int64_t inference_start_timestamp_us = detection
        ? detection->inference_start_timestamp_us : 0;
    const int64_t inference_end_timestamp_us = detection
        ? detection->inference_end_timestamp_us : 0;
    const int64_t inference_done_timestamp_us = detection
        ? detection->inference_done_timestamp_us : 0;
    const int64_t detection_receive_timestamp_us = detection
        ? detection->receive_timestamp_us : 0;

    const int64_t lidar_delivery_latency_us = elapsed_time_us(
        lidar_callback_timestamp_us, frame.detection_match_timestamp_us);
    const int64_t trigger_dispatch_latency_us = elapsed_time_us(
        trigger_sent_timestamp_us, lidar_callback_timestamp_us);
    const int64_t trigger_transport_latency_us = elapsed_time_us(
        trigger_received_timestamp_us, trigger_sent_timestamp_us);
    const int64_t camera_capture_latency_us = elapsed_time_us(
        camera_callback_timestamp_us, trigger_command_timestamp_us);
    const int64_t image_preprocess_time_us = elapsed_time_us(
        inference_start_timestamp_us, camera_callback_timestamp_us);
    const int64_t inference_time_us = elapsed_time_us(
        inference_end_timestamp_us, inference_start_timestamp_us);
    const int64_t detection_postprocess_time_us = elapsed_time_us(
        inference_done_timestamp_us, inference_end_timestamp_us);
    const int64_t detection_transport_time_us = elapsed_time_us(
        detection_receive_timestamp_us, inference_done_timestamp_us);
    const int64_t camera_pipeline_time_us = elapsed_time_us(
        inference_done_timestamp_us, trigger_command_timestamp_us);
    const int64_t point_cloud_wait_time_us = elapsed_time_us(
        frame.deskew_start_timestamp_us, lidar_callback_timestamp_us);
    const int64_t point_cloud_publish_delay_us = elapsed_time_us(
        frame.tracking_submit_timestamp_us, frame.deskew_done_timestamp_us);
    const int64_t tracking_queue_wait_time_us = elapsed_time_us(
        tracking_dequeue_timestamp_us, frame.tracking_submit_timestamp_us);
    const int64_t end_to_end_latency_us = elapsed_time_us(
        command.generated_timestamp_us, frame.lidar_end_timestamp_us);
    int64_t software_compute_time_us = -1;
    if (frame.deskew_processing_time_us >= 0 && image_preprocess_time_us >= 0
        && inference_time_us >= 0 && detection_postprocess_time_us >= 0
        && processing_time_us >= 0)
    {
      software_compute_time_us = frame.deskew_processing_time_us + image_preprocess_time_us
          + inference_time_us + detection_postprocess_time_us + processing_time_us;
    }

    tracking_stream << result.frame_timestamp_us << ',' << completed_timestamp_us << ','
                    << processing_time_us << ',' << static_cast<int>(mode.load()) << ','
                    << (detection ? detection->receive_sequence : 0) << ','
                    << (detection ? detection->schema_version : 0) << ','
                    << (result.state_valid ? 1 : 0) << ',' << (result.numerical_ok ? 1 : 0) << ','
                    << track_lifecycle_name(result.lifecycle) << ',' << result.beta_empty << ','
                    << result.hits << ',' << result.misses << ',' << result.lost_duration_s << ','
                    << result.candidates.size() << ',' << result.mode_probabilities(0) << ','
                    << result.mode_probabilities(1);
    for (int index = 0; index < 9; ++index)
    {
      tracking_stream << ',' << result.state(index);
    }
    for (int index = 0; index < 9; ++index)
    {
      tracking_stream << ',' << result.covariance(index, index);
    }
    tracking_stream << ',' << (command.valid ? 1 : 0) << ','
                    << tracking_control_source_name(command.source) << ',' << command.target_yaw_rad
                    << ',' << command.target_pitch_rad << ',' << dropped_frames.load() << ','
                    << csv_text(result.diagnostic) << '\n';

    if (tracking_timing_stream.good())
    {
      tracking_timing_stream
          << result.frame_timestamp_us << ',' << static_cast<int>(mode.load()) << ','
          << (detection ? detection->receive_sequence : 0) << ',' << (detection_matched ? 1 : 0) << ','
          << csv_text(detection ? detection->status : "") << ',' << frame.input_point_count << ','
          << frame.output_point_count << ',' << result.candidates.size() << ','
          << (command.valid ? 1 : 0) << ',' << tracking_control_source_name(command.source) << ','
          << lidar_callback_timestamp_us << ',' << trigger_sent_timestamp_us << ','
          << trigger_received_timestamp_us << ',' << trigger_command_timestamp_us << ','
          << camera_callback_timestamp_us << ',' << inference_start_timestamp_us << ','
          << inference_end_timestamp_us << ',' << inference_done_timestamp_us << ','
          << detection_receive_timestamp_us << ',' << frame.deskew_start_timestamp_us << ','
          << frame.deskew_done_timestamp_us << ',' << frame.tracking_submit_timestamp_us << ','
          << tracking_dequeue_timestamp_us << ',' << command.generated_timestamp_us << ','
          << lidar_delivery_latency_us << ',' << trigger_dispatch_latency_us << ','
          << trigger_transport_latency_us << ',' << camera_capture_latency_us << ','
          << image_preprocess_time_us << ',' << inference_time_us << ','
          << detection_postprocess_time_us << ',' << detection_transport_time_us << ','
          << camera_pipeline_time_us << ',' << point_cloud_wait_time_us << ','
          << frame.deskew_processing_time_us << ',' << point_cloud_publish_delay_us << ','
          << tracking_queue_wait_time_us << ',' << detection_wait_time_us << ','
          << processing_time_us << ',' << software_compute_time_us << ',' << end_to_end_latency_us << '\n';
    }

    for (size_t index = 0; index < result.candidates.size(); ++index)
    {
      const Candidate3D& candidate = result.candidates[index];
      candidate_stream << result.frame_timestamp_us << ',' << index << ',' << candidate.position.x()
                       << ',' << candidate.position.y() << ',' << candidate.position.z() << ','
                       << candidate.visual_confidence << ',' << candidate.point_count << ','
                       << (candidate.from_motion ? 1 : 0) << ',' << (candidate.from_visual ? 1 : 0)
                       << ',' << candidate.projected_pixel.x() << ',' << candidate.projected_pixel.y()
                       << ',' << candidate_probability(result.mode_association_probabilities[0], index,
                                                       result.candidates.size())
                       << ',' << candidate_probability(result.mode_association_probabilities[1], index,
                                                       result.candidates.size())
                       << '\n';
    }
  }

  void tracking_loop()
  {
    while (running.load())
    {
      TrackingCloudFrame frame;
      {
        std::unique_lock<std::mutex> lock(frame_mutex);
        frame_cv.wait(lock, [this] { return !running.load() || !frame_queue.empty(); });
        if (!running.load())
        {
          break;
        }
        frame = std::move(frame_queue.front());
        frame_queue.pop_front();
      }
      const int64_t tracking_dequeue_timestamp_us = unix_time_us();

      if (mode.load() == Mode::STOPPED || !frame.cloud_world || frame.cloud_world->points.empty())
      {
        continue;
      }

      DetectionFrame exact_detection;
      const int64_t detection_key = frame.detection_match_timestamp_us > 0
          ? frame.detection_match_timestamp_us
          : frame.lidar_end_timestamp_us;
      const auto detection_wait_start = std::chrono::steady_clock::now();
      const bool detection_matched = wait_for_exact_detection(detection_key, exact_detection);
      const int64_t detection_wait_time_us =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now() - detection_wait_start).count();
      if (!running.load() || mode.load() == Mode::STOPPED)
      {
        continue;
      }

      const auto processing_start = std::chrono::steady_clock::now();
      TrackResult result;
      {
        std::lock_guard<std::mutex> lock(tracker_mutex);
        double dt = static_cast<double>(frame.lidar_end_timestamp_us
                                        - previous_frame_timestamp_us)
            * 1e-6;
        if (previous_frame_timestamp_us <= 0 || dt <= 0.0 || dt > config.maximum_frame_gap_s)
        {
          if (previous_frame_timestamp_us > 0)
          {
            tracker.reset();
          }
          dt = static_cast<double>(frame.lidar_end_timestamp_us
                                   - frame.lidar_start_timestamp_us)
              * 1e-6;
          if (!std::isfinite(dt) || dt <= 0.0 || dt > config.maximum_frame_gap_s)
          {
            dt = std::min(0.1, config.maximum_frame_gap_s);
          }
        }
        previous_frame_timestamp_us = frame.lidar_end_timestamp_us;

        const Eigen::Matrix4d T_W_C = frame.T_W_G2_end * T_G2_C;
        if (!finite_transform(T_W_C))
        {
          result.numerical_ok = false;
          result.frame_timestamp_us = frame.lidar_end_timestamp_us;
          result.diagnostic = "T_W_C 无效";
        }
        else
        {
          const DetectionFrame* usable_detection = detection_matched
                  && exact_detection.status == "ok"
              ? &exact_detection
              : nullptr;
          result = tracker.step(*frame.cloud_world, usable_detection,
                                T_W_C.inverse(), frame.lidar_end_timestamp_us, dt);
        }
      }

      TrackingCommand command = build_command(
          frame, detection_matched ? &exact_detection : nullptr, result);
      const int64_t processing_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                             std::chrono::steady_clock::now() - processing_start)
                                             .count();
      log_tracking(frame, detection_matched ? &exact_detection : nullptr, detection_matched,
                   result, command, tracking_dequeue_timestamp_us, detection_wait_time_us,
                   processing_time_us);

      if (mode.load() == Mode::ACTIVE && command.valid)
      {
        CommandCallback callback;
        {
          std::lock_guard<std::mutex> lock(callback_mutex);
          callback = command_callback;
        }
        if (callback)
        {
          callback(command);
        }
      }
    }
  }
};

TargetTrackingPipeline::TargetTrackingPipeline()
    : impl_(std::make_unique<Impl>())
{
}

TargetTrackingPipeline::~TargetTrackingPipeline()
{
  stop();
  stop_saving();
}

bool TargetTrackingPipeline::load_config(const std::string& tracker_config_path,
                                         const std::string& camera_intrinsic_path,
                                         const std::string& extrinsic_config_path)
{
  if (impl_->running.load())
  {
    std::cerr << "TargetTrackingPipeline: 运行时不允许重新加载配置" << std::endl;
    return false;
  }

  Json tracker_json;
  Json camera_json;
  Json extrinsic_json;
  if (!read_json_file(tracker_config_path, tracker_json)
      || !read_json_file(camera_intrinsic_path, camera_json)
      || !read_json_file(extrinsic_config_path, extrinsic_json))
  {
    return false;
  }

  TrackingConfig config;
  CameraCalibration camera;
  Eigen::Matrix4d T_G2_C;
  if (!load_tracker_config(tracker_json, config) || !load_camera_config(camera_json, camera)
      || !load_extrinsic_config(extrinsic_json, T_G2_C))
  {
    std::cerr << "TargetTrackingPipeline: 跟踪配置校验失败" << std::endl;
    return false;
  }

  impl_->config = config;
  impl_->camera = camera;
  impl_->T_G2_C = T_G2_C;
  {
    std::lock_guard<std::mutex> lock(impl_->tracker_mutex);
    impl_->tracker.configure(config, camera);
    impl_->tracker.reset();
  }
  impl_->configured.store(true);
  std::cout << "TargetTrackingPipeline: 配置已加载，检测端口 " << config.udp_port
            << "，检测等待 " << config.detection_wait_ms << " ms" << std::endl;
  return true;
}

bool TargetTrackingPipeline::start()
{
  if (impl_->running.load())
  {
    return true;
  }
  if (!impl_->configured.load())
  {
    std::cerr << "TargetTrackingPipeline: 请先加载配置" << std::endl;
    return false;
  }

  const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd < 0)
  {
    std::cerr << "TargetTrackingPipeline: 创建 YOLO UDP socket 失败" << std::endl;
    return false;
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = htons(static_cast<uint16_t>(impl_->config.udp_port));
  if (bind(socket_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
  {
    std::cerr << "TargetTrackingPipeline: 独占绑定 YOLO UDP 端口 " << impl_->config.udp_port
              << " 失败，请确认没有第二个接收者" << std::endl;
    close(socket_fd);
    return false;
  }

  impl_->udp_socket.store(socket_fd);
  impl_->running.store(true);
  try
  {
    impl_->receiver_thread = std::thread(&Impl::receive_loop, impl_.get());
    impl_->tracking_thread = std::thread(&Impl::tracking_loop, impl_.get());
  }
  catch (const std::exception& e)
  {
    std::cerr << "TargetTrackingPipeline: 启动线程失败: " << e.what() << std::endl;
    impl_->running.store(false);
    shutdown(socket_fd, SHUT_RDWR);
    if (impl_->receiver_thread.joinable())
    {
      impl_->receiver_thread.join();
    }
    const int remaining = impl_->udp_socket.exchange(-1);
    if (remaining >= 0)
    {
      close(remaining);
    }
    return false;
  }

  std::cout << "TargetTrackingPipeline: 已启动，独占监听 UDP " << impl_->config.udp_port
            << std::endl;
  return true;
}

void TargetTrackingPipeline::stop()
{
  if (!impl_->running.exchange(false))
  {
    return;
  }
  impl_->mode.store(Mode::STOPPED);
  impl_->frame_cv.notify_all();
  impl_->detection_cv.notify_all();
  const int socket_fd = impl_->udp_socket.load();
  if (socket_fd >= 0)
  {
    shutdown(socket_fd, SHUT_RDWR);
  }
  if (impl_->receiver_thread.joinable())
  {
    impl_->receiver_thread.join();
  }
  if (impl_->tracking_thread.joinable())
  {
    impl_->tracking_thread.join();
  }
  const int remaining = impl_->udp_socket.exchange(-1);
  if (remaining >= 0)
  {
    close(remaining);
  }
  impl_->reset_tracking_state();
  std::cout << "TargetTrackingPipeline: 已停止" << std::endl;
}

void TargetTrackingPipeline::submit_frame(const TrackingCloudFrame& frame)
{
  if (!impl_->running.load() || impl_->mode.load() == Mode::STOPPED || !frame.cloud_world
      || frame.cloud_world->points.empty() || frame.lidar_end_timestamp_us <= 0)
  {
    return;
  }
  TrackingCloudFrame queued_frame = frame;
  queued_frame.tracking_submit_timestamp_us = unix_time_us();
  {
    std::lock_guard<std::mutex> lock(impl_->frame_mutex);
    while (impl_->frame_queue.size() >= impl_->config.frame_queue_size)
    {
      impl_->frame_queue.pop_front();
      ++impl_->dropped_frames;
    }
    impl_->frame_queue.push_back(std::move(queued_frame));
  }
  impl_->frame_cv.notify_one();
}

void TargetTrackingPipeline::set_mode(Mode mode)
{
  const Mode previous = impl_->mode.exchange(mode);
  if (previous == mode)
  {
    return;
  }

  if (mode == Mode::STOPPED || previous == Mode::STOPPED)
  {
    impl_->reset_tracking_state();
  }
  impl_->frame_cv.notify_all();
  impl_->detection_cv.notify_all();
  std::cout << "TargetTrackingPipeline: 模式切换为 " << static_cast<int>(mode) << std::endl;
}

TargetTrackingPipeline::Mode TargetTrackingPipeline::get_mode() const
{
  return impl_->mode.load();
}

TrackingConfig TargetTrackingPipeline::get_config() const
{
  return impl_->config;
}

void TargetTrackingPipeline::set_command_callback(CommandCallback callback)
{
  std::lock_guard<std::mutex> lock(impl_->callback_mutex);
  impl_->command_callback = std::move(callback);
}

bool TargetTrackingPipeline::start_saving(const std::string& directory)
{
  if (impl_->saving.load())
  {
    return true;
  }

  std::string output_directory = directory;
  if (output_directory.empty())
  {
    std::string base_directory;
    std::string error;
    if (!resolve_data_base_dir(base_directory, error))
    {
      std::cerr << "TargetTrackingPipeline: 数据保存目录不可用: " << error << std::endl;
      return false;
    }
    output_directory = base_directory + "/tracking";
  }

  try
  {
    std::filesystem::create_directories(output_directory);
    fix_permissions(output_directory);
  }
  catch (const std::filesystem::filesystem_error& e)
  {
    std::cerr << "TargetTrackingPipeline: 创建日志目录失败: " << e.what() << std::endl;
    return false;
  }

  const std::string suffix = time_suffix();
  std::lock_guard<std::mutex> lock(impl_->saving_mutex);
  impl_->detection_filename = output_directory + "/detection_frames_" + suffix + ".csv";
  impl_->tracking_filename = output_directory + "/tracking_state_" + suffix + ".csv";
  impl_->candidate_filename = output_directory + "/tracking_candidates_" + suffix + ".csv";
  impl_->detection_timing_filename = output_directory + "/detection_timing_" + suffix + ".csv";
  impl_->tracking_timing_filename = output_directory + "/tracking_timing_" + suffix + ".csv";
  impl_->detection_stream.clear();
  impl_->tracking_stream.clear();
  impl_->candidate_stream.clear();
  impl_->detection_timing_stream.clear();
  impl_->tracking_timing_stream.clear();
  impl_->detection_stream.open(impl_->detection_filename);
  impl_->tracking_stream.open(impl_->tracking_filename);
  impl_->candidate_stream.open(impl_->candidate_filename);
  if (!impl_->detection_stream.good() || !impl_->tracking_stream.good()
      || !impl_->candidate_stream.good())
  {
    std::cerr << "TargetTrackingPipeline: 无法创建跟踪 CSV" << std::endl;
    impl_->detection_stream.close();
    impl_->tracking_stream.close();
    impl_->candidate_stream.close();
    return false;
  }

  impl_->detection_timing_stream.open(impl_->detection_timing_filename);
  impl_->tracking_timing_stream.open(impl_->tracking_timing_filename);
  const bool timing_streams_ready = impl_->detection_timing_stream.good()
      && impl_->tracking_timing_stream.good();
  if (!timing_streams_ready)
  {
    std::cerr << "TargetTrackingPipeline: 警告，无法创建耗时 CSV，原跟踪日志继续保存" << std::endl;
    impl_->detection_timing_stream.close();
    impl_->tracking_timing_stream.close();
  }

  impl_->detection_stream << std::setprecision(10);
  impl_->tracking_stream << std::setprecision(10);
  impl_->candidate_stream << std::setprecision(10);
  if (timing_streams_ready)
  {
    impl_->detection_timing_stream << std::setprecision(10);
    impl_->tracking_timing_stream << std::setprecision(10);
  }
  impl_->detection_stream
      << "receive_sequence,schema_version,trigger_sequence,lidar_end_timestamp_us,"
         "source_timestamp_us,host_callback_timestamp_us,inference_done_timestamp_us,"
         "receive_timestamp_us,camera_frame_id,status,error,detection_count,detection_index,class_id,"
         "class_name,confidence,x1,y1,x2,y2\n";
  impl_->tracking_stream
      << "frame_timestamp_us,completed_timestamp_us,processing_time_us,pipeline_mode,"
         "detection_receive_sequence,detection_schema_version,state_valid,numerical_ok,lifecycle,"
         "beta_empty,hits,misses,lost_duration_s,candidate_count,mu_cv,mu_ca,"
         "px,py,pz,vx,vy,vz,ax,ay,az,Pxx,Pyy,Pzz,Pvx,Pvy,Pvz,Pax,Pay,Paz,"
         "command_valid,command_source,target_yaw_rad,target_pitch_rad,dropped_frames,diagnostic\n";
  impl_->candidate_stream
      << "frame_timestamp_us,candidate_index,x,y,z,visual_confidence,point_count,"
         "from_motion,from_visual,pixel_u,pixel_v,beta_cv,beta_ca\n";
  if (timing_streams_ready)
  {
    impl_->detection_timing_stream
        << "receive_sequence,schema_version,trigger_sequence,lidar_end_timestamp_us,status,error,"
           "camera_frame_id,lidar_callback_timestamp_us,trigger_sent_timestamp_us,"
           "trigger_received_timestamp_us,trigger_command_timestamp_us,camera_callback_timestamp_us,"
           "inference_start_timestamp_us,inference_end_timestamp_us,inference_done_timestamp_us,"
           "detection_receive_timestamp_us,lidar_delivery_latency_us,trigger_dispatch_latency_us,"
           "trigger_transport_latency_us,camera_capture_latency_us,image_preprocess_time_us,"
           "inference_time_us,detection_postprocess_time_us,detection_transport_time_us,"
           "camera_pipeline_time_us\n";
    impl_->tracking_timing_stream
        << "frame_timestamp_us,pipeline_mode,detection_receive_sequence,detection_matched,"
           "detection_status,input_point_count,output_point_count,candidate_count,command_valid,"
           "command_source,lidar_callback_timestamp_us,trigger_sent_timestamp_us,"
           "trigger_received_timestamp_us,trigger_command_timestamp_us,camera_callback_timestamp_us,"
           "inference_start_timestamp_us,inference_end_timestamp_us,inference_done_timestamp_us,"
           "detection_receive_timestamp_us,deskew_start_timestamp_us,deskew_done_timestamp_us,"
           "tracking_submit_timestamp_us,tracking_dequeue_timestamp_us,command_generated_timestamp_us,"
           "lidar_delivery_latency_us,trigger_dispatch_latency_us,trigger_transport_latency_us,"
           "camera_capture_latency_us,image_preprocess_time_us,inference_time_us,"
           "detection_postprocess_time_us,detection_transport_time_us,camera_pipeline_time_us,"
           "point_cloud_wait_time_us,point_cloud_processing_time_us,point_cloud_publish_delay_us,"
           "tracking_queue_wait_time_us,detection_wait_time_us,tracking_processing_time_us,"
           "software_compute_time_us,end_to_end_latency_us\n";
  }
  impl_->saving.store(true);
  std::cout << "TargetTrackingPipeline: 开始保存跟踪日志到 " << output_directory << std::endl;
  return true;
}

void TargetTrackingPipeline::stop_saving()
{
  if (!impl_->saving.exchange(false))
  {
    return;
  }
  std::lock_guard<std::mutex> lock(impl_->saving_mutex);
  impl_->detection_stream.close();
  impl_->tracking_stream.close();
  impl_->candidate_stream.close();
  impl_->detection_timing_stream.close();
  impl_->tracking_timing_stream.close();
  fix_permissions(impl_->detection_filename);
  fix_permissions(impl_->tracking_filename);
  fix_permissions(impl_->candidate_filename);
  fix_permissions(impl_->detection_timing_filename);
  fix_permissions(impl_->tracking_timing_filename);
  std::cout << "TargetTrackingPipeline: 跟踪日志已保存" << std::endl;
}
