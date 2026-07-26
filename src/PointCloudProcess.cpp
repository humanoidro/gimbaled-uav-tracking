/**
 * @file PointCloudProcess.cpp
 * @brief 点云滤波、去畸变与坐标变换实现。
 *        Point-cloud filtering, deskewing, and transformation.
 */

#include "PointCloudProcess.h"

#include "AppConfig.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

namespace
{
std::string make_session_name()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time;
    localtime_r(&now_time, &local_time);

    std::stringstream stream;
    stream << std::setfill('0')
           << std::setw(4) << (local_time.tm_year + 1900)
           << std::setw(2) << (local_time.tm_mon + 1)
           << std::setw(2) << local_time.tm_mday << "_"
           << std::setw(2) << local_time.tm_hour
           << std::setw(2) << local_time.tm_min
           << std::setw(2) << local_time.tm_sec;
    return stream.str();
}

std::string make_cloud_filename(
    const std::string& directory,
    const std::string& prefix,
    uint32_t frame_id)
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    const auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    const auto milliseconds = now_ms.time_since_epoch().count() % 1000;
    std::tm local_time;
    localtime_r(&now_time, &local_time);

    std::stringstream stream;
    stream << directory << "/" << prefix << "_"
           << std::setfill('0')
           << std::setw(4) << (local_time.tm_year + 1900)
           << std::setw(2) << (local_time.tm_mon + 1)
           << std::setw(2) << local_time.tm_mday << "_"
           << std::setw(2) << local_time.tm_hour
           << std::setw(2) << local_time.tm_min
           << std::setw(2) << local_time.tm_sec << "_"
           << std::setw(3) << milliseconds << "_"
           << std::setw(6) << frame_id << ".pcd";
    return stream.str();
}
}

PointCloudProcess::PointCloudProcess(
    LidarProcess& lidar,
    ServoController& servo,
    GimbalPoseEstimator* gimbal)
    : lidar_(lidar),
      servo_(servo),
      gimbal_(gimbal)
{
}

PointCloudProcess::~PointCloudProcess()
{
    stop();
}

bool PointCloudProcess::init()
{
    if (!gimbal_)
    {
        std::cerr << "PointCloudProcess: GimbalPoseEstimator 未初始化" << std::endl;
        return false;
    }

    lidar_.set_data_callback([this]
    {
        on_new_data_available();
    });
    servo_.set_servo_position_update_callback([this]
    {
        if (gimbal_)
        {
            gimbal_->refresh_history();
        }
        queue_condition_.notify_one();
    });
    return true;
}

void PointCloudProcess::start()
{
    if (worker_thread_.joinable())
    {
        return;
    }
    stop_flag_.store(false);
    worker_thread_ = std::thread(&PointCloudProcess::processing_loop, this);
}

void PointCloudProcess::stop()
{
    lidar_.set_data_callback(nullptr);
    servo_.set_servo_position_update_callback(nullptr);
    stop_flag_.store(true);
    queue_condition_.notify_all();

    if (worker_thread_.joinable())
    {
        worker_thread_.join();
    }
}

void PointCloudProcess::set_tracking_frame_callback(TrackingFrameCallback callback)
{
    std::lock_guard<std::mutex> lock(tracking_callback_mutex_);
    tracking_frame_callback_ = std::move(callback);
}

void PointCloudProcess::on_new_data_available()
{
    PointCloudPtr cloud;
    if (!lidar_.get_point_cloud(cloud))
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (process_queue_.size() >= MAX_QUEUE_SIZE)
        {
            process_queue_.pop();
        }
        process_queue_.push(std::move(cloud));
    }
    queue_condition_.notify_one();
}

void PointCloudProcess::processing_loop()
{
    while (!stop_flag_.load())
    {
        PointCloudPtr cloud;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_condition_.wait(lock, [this]
            {
                return !process_queue_.empty() || stop_flag_.load();
            });
            if (stop_flag_.load())
            {
                break;
            }

            while (!stop_flag_.load() && !process_queue_.empty())
            {
                cloud = process_queue_.front();
                if (can_process(cloud))
                {
                    process_queue_.pop();
                    break;
                }
                queue_condition_.wait(lock);
            }
        }

        if (!cloud || stop_flag_.load())
        {
            continue;
        }

        int64_t detection_match_timestamp_us = 0;
        if (!cloud->points.empty() && std::isfinite(cloud->points.back().timestamp))
        {
            detection_match_timestamp_us = static_cast<int64_t>(
                std::llround(cloud->points.back().timestamp * 1000000.0));
        }
        process_cloud(cloud, detection_match_timestamp_us);
    }
}

void PointCloudProcess::process_cloud(
    const PointCloudPtr& cloud,
    int64_t detection_match_timestamp_us)
{
    if (!cloud || cloud->points.empty())
    {
        return;
    }

    if (saving_.load())
    {
        if (!timestamp_offset_initialized_.load())
        {
            timestamp_offset_ = cloud->points.front().timestamp;
            timestamp_offset_initialized_.store(true);
        }

        const uint32_t frame_id = frame_counter_.fetch_add(1);
        const std::string filename =
            make_cloud_filename(raw_save_directory_, "cloud", frame_id);
        if (!save_point_cloud(filename, cloud))
        {
            std::cerr << "PointCloudProcess: 原始点云保存失败: "
                      << filename << std::endl;
        }
        else
        {
            fix_permissions(filename);
        }
    }

    TrackingCloudFrame tracking_frame;
    PointCloudPtr processed =
        deskew(cloud, &tracking_frame, detection_match_timestamp_us);
    if (!processed)
    {
        return;
    }

    if (saving_.load())
    {
        save_processed_cloud(processed);
    }

    TrackingFrameCallback callback;
    {
        std::lock_guard<std::mutex> lock(tracking_callback_mutex_);
        callback = tracking_frame_callback_;
    }
    if (callback)
    {
        callback(tracking_frame);
    }
}

bool PointCloudProcess::can_process(const PointCloudPtr& cloud)
{
    if (!cloud || cloud->points.empty() || !gimbal_)
    {
        return false;
    }

    std::vector<GimbalPoseHistory> history;
    gimbal_->get_gimbal_pose_history(history);
    if (history.size() < 2)
    {
        return false;
    }

    const double cloud_start_us = cloud->points.front().timestamp * 1000000.0;
    const double cloud_end_us = cloud->points.back().timestamp * 1000000.0;
    return std::isfinite(cloud_start_us)
        && std::isfinite(cloud_end_us)
        && history.front().timestamp_us < cloud_start_us
        && history.back().timestamp_us > cloud_end_us;
}

PointCloudPtr PointCloudProcess::deskew(
    const PointCloudPtr& cloud,
    TrackingCloudFrame* tracking_frame,
    int64_t detection_match_timestamp_us)
{
    const auto deskew_steady_start = std::chrono::steady_clock::now();
    const int64_t deskew_start_timestamp_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

    if (!cloud || cloud->points.empty() || !gimbal_)
    {
        return nullptr;
    }

    const double end_time_us = cloud->points.back().timestamp * 1000000.0;
    std::vector<GimbalPoseHistory> history;
    gimbal_->get_gimbal_pose_history(history);
    if (history.size() < 2)
    {
        return nullptr;
    }

    auto interpolate_gimbal =
        [](double time_us, const std::vector<GimbalPoseHistory>& samples)
        -> std::pair<bool, std::pair<double, double>>
    {
        auto comparator = [](const GimbalPoseHistory& sample, int64_t timestamp_us)
        {
            return sample.timestamp_us < timestamp_us;
        };
        const auto iterator = std::lower_bound(
            samples.begin(), samples.end(),
            static_cast<int64_t>(time_us), comparator);
        if (iterator == samples.end() || iterator == samples.begin())
        {
            return {false, {0.0, 0.0}};
        }

        const auto& previous = *(iterator - 1);
        const auto& next = *iterator;
        const double interval =
            static_cast<double>(next.timestamp_us - previous.timestamp_us);
        if (!std::isfinite(interval) || interval <= 0.0)
        {
            return {false, {0.0, 0.0}};
        }
        const double ratio = (time_us - previous.timestamp_us) / interval;
        return {
            true,
            {
                previous.yaw + ratio * (next.yaw - previous.yaw),
                previous.pitch + ratio * (next.pitch - previous.pitch)
            }
        };
    };

    const auto [valid_end, end_pose] =
        interpolate_gimbal(end_time_us, history);
    if (!valid_end)
    {
        return nullptr;
    }

    const double yaw_end_rad = end_pose.first;
    const double pitch_end_rad = end_pose.second;
    const Eigen::Matrix4d T_W_V_end = build_T_W_V(0.0, 0.0, 0.0);
    const Eigen::Matrix4d T_G2_L = build_T_G2_L();
    const Eigen::Matrix4d T_V_G1_end = build_T_V_G1(yaw_end_rad);
    const Eigen::Matrix4d T_G1_G2_end = build_T_G1_G2(pitch_end_rad);
    const Eigen::Matrix4d T_W_G2_end =
        T_W_V_end * T_V_G1_end * T_G1_G2_end;
    const Eigen::Matrix4d T_W_L_end = T_W_G2_end * T_G2_L;
    const Eigen::Matrix4d T_W_L_end_inverse = T_W_L_end.inverse();

    std::unordered_map<double, std::vector<size_t>> timestamp_groups;
    timestamp_groups.reserve(cloud->points.size() / 10);
    for (size_t index = 0; index < cloud->points.size(); ++index)
    {
        timestamp_groups[cloud->points[index].timestamp].push_back(index);
    }

    std::unordered_map<double, Eigen::Matrix4d> transform_cache;
    transform_cache.reserve(timestamp_groups.size());
    for (const auto& [point_time, indices] : timestamp_groups)
    {
        (void)indices;
        const auto [valid, pose] =
            interpolate_gimbal(point_time * 1000000.0, history);
        if (!valid)
        {
            continue;
        }

        const Eigen::Matrix4d T_W_L_point =
            build_T_W_V(0.0, 0.0, 0.0)
            * build_T_V_G1(pose.first)
            * build_T_G1_G2(pose.second)
            * T_G2_L;
        transform_cache[point_time] =
            T_W_L_end * (T_W_L_end_inverse * T_W_L_point);
    }

    auto processed = std::make_shared<robosense::lidar::PointCloud>();
    processed->points.reserve(cloud->points.size());
    processed->timestamp = cloud->timestamp;
    processed->seq = cloud->seq;
    processed->frame_id = "world";
    processed->is_dense = true;

    for (const auto& [timestamp, indices] : timestamp_groups)
    {
        const auto transform = transform_cache.find(timestamp);
        if (transform == transform_cache.end())
        {
            continue;
        }

        for (size_t index : indices)
        {
            const auto& source = cloud->points[index];
            const Eigen::Vector4d input(source.x, source.y, source.z, 1.0);
            const Eigen::Vector4d output = transform->second * input;
            if (!output.head<3>().allFinite())
            {
                continue;
            }

            auto point = source;
            point.x = output(0);
            point.y = output(1);
            point.z = output(2);
            processed->points.push_back(point);
        }
    }

    processed->width = static_cast<uint32_t>(processed->points.size());
    processed->height = 1;
    if (processed->points.empty())
    {
        return nullptr;
    }

    if (tracking_frame)
    {
        const auto deskew_steady_done = std::chrono::steady_clock::now();
        const int64_t deskew_done_timestamp_us =
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        tracking_frame->cloud_world = processed;
        tracking_frame->lidar_start_timestamp_us = static_cast<int64_t>(
            std::llround(cloud->points.front().timestamp * 1000000.0));
        tracking_frame->lidar_end_timestamp_us =
            static_cast<int64_t>(std::llround(end_time_us));
        tracking_frame->detection_match_timestamp_us =
            detection_match_timestamp_us > 0
            ? detection_match_timestamp_us
            : tracking_frame->lidar_end_timestamp_us;
        tracking_frame->deskew_start_timestamp_us = deskew_start_timestamp_us;
        tracking_frame->deskew_done_timestamp_us = deskew_done_timestamp_us;
        tracking_frame->deskew_processing_time_us =
            std::chrono::duration_cast<std::chrono::microseconds>(
                deskew_steady_done - deskew_steady_start).count();
        tracking_frame->input_point_count =
            static_cast<uint64_t>(cloud->points.size());
        tracking_frame->output_point_count =
            static_cast<uint64_t>(processed->points.size());
        tracking_frame->T_W_V_end = T_W_V_end;
        tracking_frame->T_W_G2_end = T_W_G2_end;
        tracking_frame->gimbal_yaw_rad = yaw_end_rad;
        tracking_frame->gimbal_pitch_rad = pitch_end_rad;
        tracking_frame->vehicle_pose_valid = false;
    }

    return processed;
}

void PointCloudProcess::set_global_saving(bool enable)
{
    if (enable && !saving_.load())
    {
        start_saving();
    }
    else if (!enable && saving_.load())
    {
        stop_saving();
    }
}

void PointCloudProcess::start_saving()
{
    std::string data_base_directory;
    std::string error_message;
    if (!resolve_data_base_dir(data_base_directory, error_message))
    {
        std::cerr << "PointCloudProcess: 数据保存目录不可用: "
                  << error_message << std::endl;
        return;
    }

    const std::string session_name = make_session_name();
    raw_save_directory_ =
        data_base_directory + "/raw_data/" + session_name;
    processed_save_directory_ =
        data_base_directory + "/processed_data/" + session_name;
    try
    {
        std::filesystem::create_directories(raw_save_directory_);
        std::filesystem::create_directories(processed_save_directory_);
        fix_permissions(raw_save_directory_);
        fix_permissions(processed_save_directory_);
    }
    catch (const std::filesystem::filesystem_error& error)
    {
        std::cerr << "PointCloudProcess: 无法创建保存目录: "
                  << error.what() << std::endl;
        return;
    }

    frame_counter_.store(1);
    timestamp_offset_initialized_.store(false);
    saving_.store(true);
}

void PointCloudProcess::stop_saving()
{
    saving_.store(false);
}

bool PointCloudProcess::save_point_cloud(
    const std::string& filename,
    const PointCloudPtr& cloud)
{
    if (!cloud)
    {
        return false;
    }

    std::ofstream output(filename, std::ios::binary);
    if (!output.is_open())
    {
        return false;
    }

    const size_t point_count = cloud->points.size();
    output << "VERSION .7\n";
    output << "# TIMESTAMP_OFFSET " << std::fixed << std::setprecision(6)
           << timestamp_offset_ << "\n";
    output << "FIELDS x y z intensity ring timestamp\n";
    output << "SIZE 4 4 4 4 2 8\n";
    output << "TYPE F F F F I F\n";
    output << "COUNT 1 1 1 1 1 1\n";
    output << "WIDTH " << point_count << "\n";
    output << "HEIGHT 1\n";
    output << "VIEWPOINT 0 0 0 1 0 0 0\n";
    output << "POINTS " << point_count << "\n";
    output << "DATA binary\n";

    for (const auto& point : cloud->points)
    {
        const float x = point.x;
        const float y = point.y;
        const float z = point.z;
        const float intensity = point.intensity;
        const int16_t ring = static_cast<int16_t>(point.ring);
        const double timestamp = point.timestamp - timestamp_offset_;
        output.write(reinterpret_cast<const char*>(&x), sizeof(x));
        output.write(reinterpret_cast<const char*>(&y), sizeof(y));
        output.write(reinterpret_cast<const char*>(&z), sizeof(z));
        output.write(reinterpret_cast<const char*>(&intensity), sizeof(intensity));
        output.write(reinterpret_cast<const char*>(&ring), sizeof(ring));
        output.write(reinterpret_cast<const char*>(&timestamp), sizeof(timestamp));
    }
    return output.good();
}

void PointCloudProcess::save_processed_cloud(const PointCloudPtr& cloud)
{
    if (!cloud || cloud->points.empty())
    {
        return;
    }

    const std::string filename = make_cloud_filename(
        processed_save_directory_, "processed_cloud", frame_counter_.load());
    if (!save_point_cloud(filename, cloud))
    {
        std::cerr << "PointCloudProcess: 处理后点云保存失败: "
                  << filename << std::endl;
        return;
    }
    fix_permissions(filename);
}

void PointCloudProcess::fix_permissions(const std::string& path)
{
    const char* sudo_uid = std::getenv("SUDO_UID");
    const char* sudo_gid = std::getenv("SUDO_GID");
    if (sudo_uid && sudo_gid)
    {
        const uid_t uid = static_cast<uid_t>(std::stoul(sudo_uid));
        const gid_t gid = static_cast<gid_t>(std::stoul(sudo_gid));
        if (chown(path.c_str(), uid, gid) != 0)
        {
            std::cerr << "PointCloudProcess: 无法修改文件所有权: "
                      << std::strerror(errno) << std::endl;
        }
    }

    struct stat status;
    if (stat(path.c_str(), &status) == 0)
    {
        chmod(path.c_str(), S_ISDIR(status.st_mode) ? 0775 : 0664);
    }
}

Eigen::Matrix4d PointCloudProcess::build_T_V_G1(double yaw_rad)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    const double cosine = std::cos(yaw_rad);
    const double sine = std::sin(yaw_rad);
    transform(0, 0) = cosine;
    transform(0, 1) = -sine;
    transform(0, 3) = 0.18;
    transform(1, 0) = sine;
    transform(1, 1) = cosine;
    transform(2, 3) = 0.85;
    return transform;
}

Eigen::Matrix4d PointCloudProcess::build_T_G1_G2(double pitch_rad)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    const double cosine = std::cos(pitch_rad);
    const double sine = std::sin(pitch_rad);
    transform(0, 0) = cosine;
    transform(0, 2) = sine;
    transform(2, 0) = -sine;
    transform(2, 2) = cosine;
    transform(2, 3) = 0.062;
    return transform;
}

Eigen::Matrix4d PointCloudProcess::build_T_G2_L()
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform(0, 3) = 0.018;
    transform(2, 3) = 0.072;
    return transform;
}

Eigen::Matrix4d PointCloudProcess::build_T_W_V(
    double x,
    double y,
    double yaw)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    transform(0, 0) = cosine;
    transform(0, 1) = -sine;
    transform(0, 3) = x;
    transform(1, 0) = sine;
    transform(1, 1) = cosine;
    transform(1, 3) = y;
    return transform;
}
