/**
 * @file GimbalPoseEstimator.h
 * @brief 基于 IMU 与编码器的云台关节角估计接口。
 *        IMU-encoder gimbal joint-angle estimation interface.
 */

#pragma once

#include <deque>
#include <vector>
#include <mutex>
#include <functional>
#include <chrono>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include "common.h"
#include "ServoController.h"
#include "IMUReader.h"

struct GimbalPoseRecord {
    int64_t timestamp_us;
    double yaw_rad;
    double pitch_rad;
    double gyro_x;
    double gyro_y;
    double gyro_z;
    bool is_update; // 编码器更新标记 / Encoder-update flag
};

/**
 * @brief 融合双 IMU 与编码器估计云台关节角。
 *        Estimates gimbal joint angles from dual IMUs and encoders.
 */
class GimbalPoseEstimator {
public:
    GimbalPoseEstimator(ServoController& servo, IMUReader& gimbal_imu, IMUReader& vehicle_imu);
    ~GimbalPoseEstimator();

    void get_gimbal_pose_history(std::vector<GimbalPoseHistory>& history) const;
    void refresh_history();
    void refresh_imu_history();

    /// 编码器观测更新 / Encoder measurement update.
    void update_step(double observed_yaw, double observed_pitch);

    void start_saving();
    void stop_saving();

private:
    ServoController& m_servo;
    IMUReader& m_imu;
    IMUReader& m_vehicle_imu;

    mutable std::mutex m_history_mutex;
    mutable std::mutex m_ekf_mutex;

    std::vector<ServoPositionHistory> m_yaw_history_cache;
    std::vector<ServoPositionHistory> m_pitch_history_cache;
    std::vector<IMUDataRecord> m_imu_history_cache;

    static const size_t MAX_GIMBAL_POSE_HISTORY_SIZE = 1000;
    std::vector<GimbalPoseHistory> m_gimbal_pose_history;

    GimbalPose m_current_pose;
    IMUDataRecord m_last_imu_data{};
    int64_t m_last_imu_timestamp_us{0};

    std::thread m_predict_thread;
    std::atomic<bool> m_predict_running{false};
    mutable std::mutex m_predict_mutex;
    std::condition_variable m_predict_cv;

    void predict_thread_func();
    void init_kalman_filter();

    Eigen::Matrix3d compute_V_R_G2(double yaw_rad, double pitch_rad) const;

    Eigen::Vector3d compute_rel_angular_velocity(
        const IMUDataRecord& gimbal_imu_data,
        double yaw_rad, double pitch_rad) const;

    Eigen::Vector2d compute_joint_rates(
        const Eigen::Vector3d& V_omega_rel,
        double yaw_rad) const;

    cv::Mat compute_state_transition_jacobian(
        const Eigen::Vector3d& V_omega_rel,
        double yaw_rad) const;

    void ekf_predict(const IMUDataRecord& imu_data, double dt);

    // 固定 IMU 旋转 / Fixed IMU rotations.
    Eigen::Matrix3d m_G2_R_IG;
    Eigen::Matrix3d m_V_R_IV;

    // EKF 状态矩阵 / EKF state matrices.
    cv::Mat m_state;
    cv::Mat m_P;
    cv::Mat m_R;
    cv::Mat m_Q_base;
    cv::Mat m_H;
    cv::Mat m_K;
    cv::Mat m_I;

    // 数据保存 / Data recording.
    std::atomic<bool> m_saving_flag{false};
    std::deque<GimbalPoseRecord> m_save_buffer;
    mutable std::mutex m_save_mutex;
    std::string m_save_directory;

    void save_to_csv();
    void fix_permissions(const std::string& path);
};
