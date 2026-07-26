/**
 * @file IMUReader.h
 * @brief 串口 IMU 采集与时间戳缓存接口。
 *        Serial IMU acquisition and timestamped buffering.
 */

#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <vector>
#include <deque>
#include <functional>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cerrno>
#include <cstring>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <limits>

struct IMUDataRecord {
    float roll_speed;
    float pitch_speed;
    float heading_speed;
    float acc_x;
    float acc_y;
    float acc_z;
    float roll;
    float pitch;
    float yaw;
    float q1, q2, q3, q4;
    int32_t timestamp_us;
    int32_t frame_count;
    int64_t unix_timestamp_us;
    std::chrono::system_clock::time_point timestamp;
};

class IMUReader
{
public:
    IMUReader(const std::string &port, int baud_rate, const std::string &save_prefix = "imu");

    ~IMUReader();

    IMUReader(const IMUReader &) = delete;
    IMUReader &operator=(const IMUReader &) = delete;

    bool is_running() const;

    void start_saving();
    
    void stop_saving();

    void set_imu_data_update_callback(std::function<void()> callback);

    bool get_latest_history(IMUDataRecord& history);

    size_t get_all_history(std::vector<IMUDataRecord>& history_list);

    bool calibrate_angular_velocity(double stability_threshold = 0.01, int max_retries = 20);

private:
    void read_loop();
    void open_and_configure_port();
    void close_port();
    
    void save_to_csv();
    
    void fix_permissions(const std::string& path);

    std::string m_port;
    int m_baud_rate;
    int m_serial_fd = -1;

    std::thread m_read_thread;
    std::atomic<bool> m_is_running{false};
    
    std::deque<IMUDataRecord> m_imu_data_buffer;
    std::mutex m_buffer_mutex;
    std::atomic<bool> m_saving_flag{false};
    std::string m_save_directory;
    std::string m_save_prefix{"imu"};

    std::function<void()> m_imu_data_update_callback;
    std::mutex m_callback_mutex;

    std::deque<IMUDataRecord> m_imu_history;
    static const size_t MAX_HISTORY_SIZE = 500;
    mutable std::mutex m_history_mutex;

    double m_roll_speed_offset{0.0};
    double m_pitch_speed_offset{0.0};
    double m_heading_speed_offset{0.0};

    float m_latest_acc_x{std::numeric_limits<float>::quiet_NaN()};
    float m_latest_acc_y{std::numeric_limits<float>::quiet_NaN()};
    float m_latest_acc_z{std::numeric_limits<float>::quiet_NaN()};

};
