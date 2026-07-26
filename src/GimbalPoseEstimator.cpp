/**
 * @file GimbalPoseEstimator.cpp
 * @brief IMU-编码器云台关节角估计实现。
 *        IMU-encoder gimbal joint-angle estimation.
 */

#include "GimbalPoseEstimator.h"
#include "ServoController.h"
#include "IMUReader.h"
#include "AppConfig.h"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sys/stat.h>

GimbalPoseEstimator::GimbalPoseEstimator(ServoController& servo, IMUReader& gimbal_imu, IMUReader& vehicle_imu)
    : m_servo(servo), m_imu(gimbal_imu), m_vehicle_imu(vehicle_imu)
{
    m_G2_R_IG = Eigen::Matrix3d::Identity();
    m_G2_R_IG(1, 1) = -1.0;
    m_G2_R_IG(2, 2) = -1.0;

    m_V_R_IV = Eigen::Matrix3d::Identity();
    m_V_R_IV(1, 1) = -1.0;
    m_V_R_IV(2, 2) = -1.0;

    const int max_attempts = 50;
    const int retry_interval_ms = 10;

    std::cout << "GimbalPoseEstimator: 开始初始化角度..." << std::endl;

    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
        uint16_t yaw_pos, pitch_pos;
        if (m_servo.get_cached_position_dual(1, yaw_pos, 2, pitch_pos))
        {
            double yaw_deg = compute_yaw_angle(yaw_pos);
            double pitch_deg = compute_pitch_angle(pitch_pos);
            m_current_pose.yaw = deg_to_rad(yaw_deg);
            m_current_pose.pitch = deg_to_rad(pitch_deg);

            auto now = std::chrono::system_clock::now();
            auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                now.time_since_epoch()).count();
            m_current_pose.timestamp_us = now_us;

            std::cout << "GimbalPoseEstimator: 初始化成功" << std::endl;
            std::cout << "  舵机原始值: yaw=" << yaw_pos << ", pitch=" << pitch_pos << std::endl;
            std::cout << "  角度(度): yaw=" << yaw_deg << "°, pitch=" << pitch_deg << "°" << std::endl;
            std::cout << "  角度(弧度): yaw=" << m_current_pose.yaw << " rad, pitch=" << m_current_pose.pitch << " rad" << std::endl;

            m_last_imu_timestamp_us = m_current_pose.timestamp_us;

            m_last_imu_data = IMUDataRecord{};
            m_last_imu_data.unix_timestamp_us = m_current_pose.timestamp_us;

            init_kalman_filter();

            m_predict_running.store(true);
            m_predict_thread = std::thread(&GimbalPoseEstimator::predict_thread_func, this);

            // 初始化完成后注册回调 / Register callbacks after initialization.
            m_servo.set_servo_position_update_callback([this]() { this->refresh_history(); });
            m_imu.set_imu_data_update_callback([this]() { this->refresh_imu_history(); });

            return;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms));
    }

    throw std::runtime_error("无法获取舵机位置，舵机可能未就绪");
}

GimbalPoseEstimator::~GimbalPoseEstimator()
{
    m_predict_running.store(false);
    m_predict_cv.notify_all();
    if (m_predict_thread.joinable())
    {
        m_predict_thread.join();
    }

    m_servo.set_servo_position_update_callback(nullptr);
    m_imu.set_imu_data_update_callback(nullptr);
}

void GimbalPoseEstimator::refresh_history()
{
    std::vector<ServoPositionHistory> yaw_hist, pitch_hist;
    m_servo.get_all_history(1, yaw_hist);
    m_servo.get_all_history(2, pitch_hist);

    const bool have_observation = !yaw_hist.empty() && !pitch_hist.empty();
    double observed_yaw_deg = 0.0;
    double observed_pitch_deg = 0.0;
    if (have_observation)
    {
        observed_yaw_deg = compute_yaw_angle(yaw_hist.back().position);
        observed_pitch_deg = compute_pitch_angle(pitch_hist.back().position);
    }

    {
        std::lock_guard<std::mutex> lock(m_history_mutex);
        m_yaw_history_cache = std::move(yaw_hist);
        m_pitch_history_cache = std::move(pitch_hist);
    }

    if (have_observation)
    {
        update_step(deg_to_rad(observed_yaw_deg), deg_to_rad(observed_pitch_deg));
    }
}

void GimbalPoseEstimator::refresh_imu_history()
{
    std::vector<IMUDataRecord> imu_hist;
    m_imu.get_all_history(imu_hist);

    {
        std::lock_guard<std::mutex> lock(m_history_mutex);
        m_imu_history_cache = std::move(imu_hist);
    }

    m_predict_cv.notify_one();
}

void GimbalPoseEstimator::predict_thread_func()
{
    while (m_predict_running.load())
    {
        std::unique_lock<std::mutex> lock(m_predict_mutex);
        m_predict_cv.wait(lock);

        if (!m_predict_running.load())
        {
            break;
        }

        std::vector<IMUDataRecord> imu_hist;
        {
            std::lock_guard<std::mutex> hist_lock(m_history_mutex);
            imu_hist = m_imu_history_cache;
        }

        if (imu_hist.empty())
        {
            continue;
        }

        const IMUDataRecord& latest_imu = imu_hist.back();

        if (m_last_imu_timestamp_us == 0)
        {
            m_last_imu_timestamp_us = latest_imu.unix_timestamp_us;
            m_last_imu_data = latest_imu;
            continue;
        }

        double dt = (latest_imu.unix_timestamp_us - m_last_imu_timestamp_us) / 1e6;

        if (dt <= 0 || dt > 0.5)
        {
            std::cerr << "警告: 无效的时间间隔 dt=" << dt << "秒，跳过此次预测" << std::endl;
            continue;
        }

        {
            std::lock_guard<std::mutex> ekf_lock(m_ekf_mutex);
            ekf_predict(latest_imu, dt);
        }

        m_last_imu_data = latest_imu;
        m_last_imu_timestamp_us = latest_imu.unix_timestamp_us;

    }
}

void GimbalPoseEstimator::ekf_predict(const IMUDataRecord& imu_data, double dt)
{
    double theta_pan_hat  = m_state.at<double>(0, 0);
    double theta_tilt_hat = m_state.at<double>(1, 0);

    Eigen::Vector3d V_omega_rel = compute_rel_angular_velocity(imu_data, theta_pan_hat, theta_tilt_hat);

    Eigen::Vector2d f_k = compute_joint_rates(V_omega_rel, theta_pan_hat);

    double theta_pan_mid  = theta_pan_hat  + 0.5 * dt * f_k(0);
    double theta_tilt_mid = theta_tilt_hat + 0.5 * dt * f_k(1);

    Eigen::Vector3d V_omega_rel_mid = compute_rel_angular_velocity(imu_data, theta_pan_mid, theta_tilt_mid);

    Eigen::Vector2d f_mid = compute_joint_rates(V_omega_rel_mid, theta_pan_mid);

    cv::Mat x_bar = cv::Mat_<double>(2, 1);
    x_bar.at<double>(0, 0) = theta_pan_hat  + dt * f_mid(0);
    x_bar.at<double>(1, 0) = theta_tilt_hat + dt * f_mid(1);

    cv::Mat J = compute_state_transition_jacobian(V_omega_rel_mid, theta_pan_mid);
    cv::Mat F_G = m_I + dt * J;

    cv::Mat Q_scaled = m_Q_base * dt;
    m_P = F_G * m_P * F_G.t() + Q_scaled;

    m_state = x_bar.clone();

    // 使用 IMU 时间戳对齐点云 / Use IMU timestamps for point-cloud alignment.
    const int64_t timestamp_us = imu_data.unix_timestamp_us;

    {
        std::lock_guard<std::mutex> lock(m_history_mutex);
        m_gimbal_pose_history.push_back({
            m_state.at<double>(0, 0),
            m_state.at<double>(1, 0),
            timestamp_us
        });

        if (m_gimbal_pose_history.size() > MAX_GIMBAL_POSE_HISTORY_SIZE) {
            m_gimbal_pose_history.erase(m_gimbal_pose_history.begin());
        }
    }

    if (m_saving_flag.load())
    {
        std::lock_guard<std::mutex> lock(m_save_mutex);
        m_save_buffer.push_back({
            timestamp_us,
            m_state.at<double>(0, 0),
            m_state.at<double>(1, 0),
            imu_data.roll_speed,
            imu_data.pitch_speed,
            imu_data.heading_speed,
            false
        });
    }
}

Eigen::Vector3d GimbalPoseEstimator::compute_rel_angular_velocity(
    const IMUDataRecord& gimbal_imu_data,
    double yaw_rad, double pitch_rad) const
{
    Eigen::Vector3d omega_IG(
        gimbal_imu_data.roll_speed,
        gimbal_imu_data.pitch_speed,
        gimbal_imu_data.heading_speed);

    Eigen::Vector3d omega_G2 = m_G2_R_IG * omega_IG;

    Eigen::Matrix3d V_R_G2 = compute_V_R_G2(yaw_rad, pitch_rad);
    Eigen::Vector3d V_omega_rel = V_R_G2 * omega_G2;

    IMUDataRecord veh_data;
    if (m_vehicle_imu.get_latest_history(veh_data))
    {
        Eigen::Vector3d omega_IV(
            veh_data.roll_speed,
            veh_data.pitch_speed,
            veh_data.heading_speed);
        V_omega_rel -= m_V_R_IV * omega_IV;
    }

    return V_omega_rel;
}

Eigen::Matrix3d GimbalPoseEstimator::compute_V_R_G2(double yaw_rad, double pitch_rad) const
{
    double cy = std::cos(yaw_rad);
    double sy = std::sin(yaw_rad);
    double cp = std::cos(pitch_rad);
    double sp = std::sin(pitch_rad);

    Eigen::Matrix3d R;
    R << cy * cp, -sy, cy * sp,
         sy * cp,  cy, sy * sp,
         -sp,      0,  cp;
    return R;
}

Eigen::Vector2d GimbalPoseEstimator::compute_joint_rates(
    const Eigen::Vector3d& V_omega_rel,
    double yaw_rad) const
{
    double sy = std::sin(yaw_rad);
    double cy = std::cos(yaw_rad);

    double pan_rate  = V_omega_rel(2);
    double tilt_rate = -sy * V_omega_rel(0) + cy * V_omega_rel(1);

    return Eigen::Vector2d(pan_rate, tilt_rate);
}

cv::Mat GimbalPoseEstimator::compute_state_transition_jacobian(
    const Eigen::Vector3d& V_omega_rel,
    double yaw_rad) const
{
    double sy = std::sin(yaw_rad);
    double cy = std::cos(yaw_rad);

    cv::Mat J = cv::Mat::zeros(2, 2, CV_64F);
    J.at<double>(1, 0) = -cy * V_omega_rel(0) - sy * V_omega_rel(1);

    return J;
}

void GimbalPoseEstimator::update_step(double observed_yaw, double observed_pitch)
{
    std::lock_guard<std::mutex> lock(m_ekf_mutex);

    cv::Mat z = (cv::Mat_<double>(2, 1) << observed_yaw, observed_pitch);

    cv::Mat y = z - m_H * m_state;

    cv::Mat S = m_H * m_P * m_H.t() + m_R;

    cv::Mat S_inv = S.inv();

    m_K = m_P * m_H.t() * S_inv;

    m_state = m_state + m_K * y;

    m_P = (m_I - m_K * m_H) * m_P;

    auto now = std::chrono::system_clock::now();
    auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count();

    {
        std::lock_guard<std::mutex> lock(m_history_mutex);
        m_gimbal_pose_history.push_back({
            m_state.at<double>(0, 0),
            m_state.at<double>(1, 0),
            timestamp_us
        });

        if (m_gimbal_pose_history.size() > MAX_GIMBAL_POSE_HISTORY_SIZE) {
            m_gimbal_pose_history.erase(m_gimbal_pose_history.begin());
        }
    }

    if (m_saving_flag.load())
    {
        std::lock_guard<std::mutex> lock(m_save_mutex);
        m_save_buffer.push_back({
            timestamp_us,
            m_state.at<double>(0, 0),
            m_state.at<double>(1, 0),
            0.0, 0.0, 0.0,
            true
        });
    }
}

void GimbalPoseEstimator::init_kalman_filter()
{
    m_state = cv::Mat_<double>(2, 1);
    m_state.at<double>(0, 0) = m_current_pose.yaw;
    m_state.at<double>(1, 0) = m_current_pose.pitch;

    double P_init = deg_to_rad(1.0);
    P_init *= P_init;
    m_P = cv::Mat::eye(2, 2, CV_64F) * P_init;

    double R_deg = 3.0;
    double R_rad = deg_to_rad(R_deg);
    R_rad *= R_rad;
    m_R = cv::Mat::eye(2, 2, CV_64F) * R_rad;

    m_Q_base = cv::Mat::eye(2, 2, CV_64F) * 1.5e-5;

    m_H = cv::Mat::eye(2, 2, CV_64F);

    m_K = cv::Mat::zeros(2, 2, CV_64F);

    m_I = cv::Mat::eye(2, 2, CV_64F);
}

void GimbalPoseEstimator::get_gimbal_pose_history(std::vector<GimbalPoseHistory>& history) const
{
    std::lock_guard<std::mutex> lock(m_history_mutex);
    history = m_gimbal_pose_history;
}

void GimbalPoseEstimator::start_saving()
{
    if (m_saving_flag.load()) return;

    std::string data_base_dir;
    std::string error_message;
    if (!resolve_data_base_dir(data_base_dir, error_message))
    {
        std::cerr << "GimbalPoseEstimator: 数据保存目录不可用: " << error_message << std::endl;
        return;
    }

    m_save_directory = data_base_dir + "/gimbal_pose";
    try
    {
        std::filesystem::create_directories(m_save_directory);
        fix_permissions(m_save_directory);
        std::filesystem::path p(m_save_directory);
        while (p.has_parent_path() && p != p.root_path())
        {
            p = p.parent_path();
            fix_permissions(p.string());
        }
    }
    catch (const std::filesystem::filesystem_error& e)
    {
        std::cerr << "GimbalPoseEstimator: 无法创建保存目录: " << e.what() << std::endl;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(m_save_mutex);
        m_save_buffer.clear();
    }
    m_saving_flag.store(true);
    std::cout << "GimbalPoseEstimator: 开始保存EKF姿态数据到: " << m_save_directory << std::endl;
}

void GimbalPoseEstimator::stop_saving()
{
    size_t count = 0;
    {
        std::lock_guard<std::mutex> lock(m_save_mutex);
        count = m_save_buffer.size();
    }
    if (count > 0) save_to_csv();
    {
        std::lock_guard<std::mutex> lock(m_save_mutex);
        m_save_buffer.clear();
    }
    m_saving_flag.store(false);
    std::cout << "GimbalPoseEstimator: 停止保存EKF姿态数据，共 " << count << " 条" << std::endl;
}

void GimbalPoseEstimator::save_to_csv()
{
    std::deque<GimbalPoseRecord> records;
    {
        std::lock_guard<std::mutex> lock(m_save_mutex);
        records.swap(m_save_buffer);
    }
    if (records.empty()) return;

    auto now = std::chrono::system_clock::now();
    auto now_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto ms = now_ms.time_since_epoch().count() % 1000;
    std::tm tm_buf;
    localtime_r(&now_t, &tm_buf);

    std::stringstream ss;
    ss << m_save_directory << "/gimbal_pose_"
       << std::setfill('0') << std::setw(4) << (tm_buf.tm_year + 1900)
       << std::setw(2) << (tm_buf.tm_mon + 1)
       << std::setw(2) << tm_buf.tm_mday << "_"
       << std::setw(2) << tm_buf.tm_hour
       << std::setw(2) << tm_buf.tm_min
       << std::setw(2) << tm_buf.tm_sec << "_"
       << std::setw(3) << ms << ".csv";
    std::string filename = ss.str();

    std::ofstream ofs(filename);
    if (!ofs.is_open()) { std::cerr << "GimbalPoseEstimator: 无法打开文件: " << filename << std::endl; return; }

    ofs << std::fixed << std::setprecision(6);
    ofs << "timestamp_us,yaw_rad,pitch_rad,gyro_x,gyro_y,gyro_z,is_update\n";
    for (const auto& r : records)
    {
        ofs << r.timestamp_us << ","
            << r.yaw_rad << "," << r.pitch_rad << ","
            << r.gyro_x << "," << r.gyro_y << "," << r.gyro_z << ","
            << (r.is_update ? 1 : 0) << "\n";
    }
    ofs.close();
    fix_permissions(filename);
    std::cout << "GimbalPoseEstimator: 姿态数据已保存到 " << filename << " (" << records.size() << " 条)" << std::endl;
}

void GimbalPoseEstimator::fix_permissions(const std::string& path)
{
    const char* sudo_uid_ptr = std::getenv("SUDO_UID");
    const char* sudo_gid_ptr = std::getenv("SUDO_GID");
    if (sudo_uid_ptr && sudo_gid_ptr)
    {
        uid_t uid = static_cast<uid_t>(std::stoul(sudo_uid_ptr));
        gid_t gid = static_cast<gid_t>(std::stoul(sudo_gid_ptr));
        if (chown(path.c_str(), uid, gid) != 0) {}
        struct stat st;
        if (stat(path.c_str(), &st) == 0)
            chmod(path.c_str(), S_ISDIR(st.st_mode) ? 0775 : 0664);
    }
}
