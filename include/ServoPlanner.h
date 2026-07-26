/**
 * @file ServoPlanner.h
 * @brief 云台指令执行与速率限制接口。
 *        Gimbal command execution and rate limiting.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "ServoController.h"
#include "TrackingTypes.h"

struct ImmPdafControlTimingRecord
{
    uint64_t sequence;
    int64_t frame_timestamp_us;
    int64_t command_generated_timestamp_us;
    int64_t planner_applied_timestamp_us;
    int64_t command_to_planner_time_us;
    int64_t frame_to_planner_time_us;
    int command_source;
    int lifecycle;
    double command_age_s;
    uint16_t current_encoder_yaw;
    uint16_t current_encoder_pitch;
    double requested_encoder_yaw;
    double requested_encoder_pitch;
    uint16_t applied_encoder_yaw;
    uint16_t applied_encoder_pitch;
    uint16_t command_time_ms;
    bool rate_limited;
};

class ServoPlanner
{
public:
    enum class Mode
    {
        FIXED,
        IMM_PDAF_TRACKING
    };

    explicit ServoPlanner(ServoController& servo);
    ~ServoPlanner();

    ServoPlanner(const ServoPlanner&) = delete;
    ServoPlanner& operator=(const ServoPlanner&) = delete;

    void set_mode(Mode mode);
    void set_fixed_position(uint8_t id1, uint16_t pos1, uint16_t time1,
                            uint8_t id2, uint16_t pos2, uint16_t time2);

    /// 配置跟踪控制参数 / Configure tracking-control parameters.
    bool configure_tracking(const TrackingConfig& config);
    void move_home();

    void set_tracking_command(const TrackingCommand& command);
    void clear_tracking_command();
    void hold_current_position();

    void start();
    void stop();

    void start_saving();
    void stop_saving();

private:
    struct TrackingControlParams
    {
        uint16_t min_encoder_yaw = 0;
        uint16_t max_encoder_yaw = 4100;
        uint16_t min_encoder_pitch = 1800;
        uint16_t max_encoder_pitch = 3000;
        double max_encoder_rate = 800.0;
        uint16_t home_encoder_yaw = 2048;
        uint16_t home_encoder_pitch = 2000;
        uint16_t command_time_ms = 10;
        uint16_t home_move_time_ms = 500;
    };

    void update();
    void update_thread_func();
    void update_fixed();
    void update_imm_pdaf_tracking();
    void save_imm_pdaf_timing_to_csv();
    void fix_permissions(const std::string& path);

    ServoController& servo_;
    Mode mode_;
    std::atomic<bool> running_;
    std::atomic<bool> update_thread_should_stop_;
    std::thread update_thread_;
    std::mutex mutex_;

    uint8_t fixed_id1_;
    uint16_t fixed_pos1_;
    uint16_t fixed_time1_;
    uint8_t fixed_id2_;
    uint16_t fixed_pos2_;
    uint16_t fixed_time2_;

    TrackingControlParams control_params_;
    uint16_t current_encoder_yaw_{2048};
    uint16_t current_encoder_pitch_{1923};
    bool target_initialized_{false};
    double command_target_yaw_{2048.0};
    double command_target_pitch_{1923.0};
    std::chrono::steady_clock::time_point last_control_time_;

    TrackingCommand latest_tracking_command_;
    std::mutex tracking_command_mutex_;
    std::chrono::steady_clock::time_point tracking_command_receive_time_;

    std::atomic<bool> saving_{false};
    std::deque<ImmPdafControlTimingRecord> timing_buffer_;
    uint64_t timing_sequence_{0};
    mutable std::mutex save_mutex_;
    std::string save_directory_;
};
