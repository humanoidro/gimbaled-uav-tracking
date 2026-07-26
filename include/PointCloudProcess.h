/**
 * @file PointCloudProcess.h
 * @brief 点云滤波、去畸变与坐标变换接口。
 *        Point-cloud filtering, deskewing, and transformation.
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <Eigen/Dense>

#include "GimbalPoseEstimator.h"
#include "LidarProcess.h"
#include "ServoController.h"
#include "TrackingTypes.h"

class PointCloudProcess
{
public:
    using TrackingFrameCallback = std::function<void(const TrackingCloudFrame&)>;

    PointCloudProcess(
        LidarProcess& lidar,
        ServoController& servo,
        GimbalPoseEstimator* gimbal);
    ~PointCloudProcess();

    PointCloudProcess(const PointCloudProcess&) = delete;
    PointCloudProcess& operator=(const PointCloudProcess&) = delete;

    bool init();
    void start();
    void stop();
    void set_global_saving(bool enable);
    void set_tracking_frame_callback(TrackingFrameCallback callback);

private:
    void on_new_data_available();
    void processing_loop();
    void process_cloud(
        const PointCloudPtr& cloud,
        int64_t detection_match_timestamp_us);
    bool can_process(const PointCloudPtr& cloud);
    PointCloudPtr deskew(
        const PointCloudPtr& cloud,
        TrackingCloudFrame* tracking_frame,
        int64_t detection_match_timestamp_us);

    void start_saving();
    void stop_saving();
    bool save_point_cloud(
        const std::string& filename,
        const PointCloudPtr& cloud);
    void save_processed_cloud(const PointCloudPtr& cloud);
    void fix_permissions(const std::string& path);

    Eigen::Matrix4d build_T_V_G1(double yaw_rad);
    Eigen::Matrix4d build_T_G1_G2(double pitch_rad);
    Eigen::Matrix4d build_T_G2_L();
    Eigen::Matrix4d build_T_W_V(double x, double y, double yaw);

    LidarProcess& lidar_;
    ServoController& servo_;
    GimbalPoseEstimator* gimbal_;

    std::thread worker_thread_;
    std::atomic<bool> stop_flag_{false};
    std::queue<PointCloudPtr> process_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_condition_;
    static constexpr size_t MAX_QUEUE_SIZE = 10;

    std::atomic<bool> saving_{false};
    std::string raw_save_directory_;
    std::string processed_save_directory_;
    std::atomic<uint32_t> frame_counter_{0};
    double timestamp_offset_{0.0};
    std::atomic<bool> timestamp_offset_initialized_{false};

    TrackingFrameCallback tracking_frame_callback_;
    std::mutex tracking_callback_mutex_;
};
