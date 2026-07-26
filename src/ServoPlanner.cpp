/**
 * @file ServoPlanner.cpp
 * @brief 云台控制指令执行实现。
 *        Gimbal control-command execution.
 */

#include "ServoPlanner.h"

#include "AppConfig.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <sys/stat.h>
#include <unistd.h>

ServoPlanner::ServoPlanner(ServoController& servo)
    : servo_(servo),
      mode_(Mode::FIXED),
      running_(false),
      update_thread_should_stop_(false),
      fixed_id1_(1),
      fixed_pos1_(2048),
      fixed_time1_(10),
      fixed_id2_(2),
      fixed_pos2_(2048),
      fixed_time2_(10),
      last_control_time_(std::chrono::steady_clock::now()),
      tracking_command_receive_time_(std::chrono::steady_clock::now())
{
}

ServoPlanner::~ServoPlanner()
{
    stop();
}

void ServoPlanner::set_mode(Mode mode)
{
    std::lock_guard<std::mutex> lock(mutex_);
    mode_ = mode;
    if (mode_ == Mode::IMM_PDAF_TRACKING)
    {
        target_initialized_ = false;
        last_control_time_ = std::chrono::steady_clock::now();
    }
    std::cout << "ServoPlanner: 模式设置为 " << static_cast<int>(mode_) << std::endl;
}

void ServoPlanner::set_fixed_position(uint8_t id1, uint16_t pos1, uint16_t time1,
                                      uint8_t id2, uint16_t pos2, uint16_t time2)
{
    std::lock_guard<std::mutex> lock(mutex_);
    fixed_id1_ = id1;
    fixed_pos1_ = pos1;
    fixed_time1_ = time1;
    fixed_id2_ = id2;
    fixed_pos2_ = pos2;
    fixed_time2_ = time2;
}

bool ServoPlanner::configure_tracking(const TrackingConfig& config)
{
    const bool valid =
        config.min_encoder_yaw >= 0 &&
        config.max_encoder_yaw <= 65535 &&
        config.min_encoder_pitch >= 0 &&
        config.max_encoder_pitch <= 65535 &&
        config.min_encoder_yaw < config.max_encoder_yaw &&
        config.min_encoder_pitch < config.max_encoder_pitch &&
        config.home_encoder_yaw >= config.min_encoder_yaw &&
        config.home_encoder_yaw <= config.max_encoder_yaw &&
        config.home_encoder_pitch >= config.min_encoder_pitch &&
        config.home_encoder_pitch <= config.max_encoder_pitch &&
        std::isfinite(config.max_encoder_rate) &&
        config.max_encoder_rate > 0.0 &&
        config.servo_command_time_ms > 0 &&
        config.servo_command_time_ms <= 65535 &&
        config.home_move_time_ms > 0 &&
        config.home_move_time_ms <= 65535;
    if (!valid)
    {
        std::cerr << "ServoPlanner: 跟踪控制参数无效" << std::endl;
        return false;
    }

    TrackingControlParams params;
    params.min_encoder_yaw = static_cast<uint16_t>(config.min_encoder_yaw);
    params.max_encoder_yaw = static_cast<uint16_t>(config.max_encoder_yaw);
    params.min_encoder_pitch = static_cast<uint16_t>(config.min_encoder_pitch);
    params.max_encoder_pitch = static_cast<uint16_t>(config.max_encoder_pitch);
    params.max_encoder_rate = config.max_encoder_rate;
    params.home_encoder_yaw = static_cast<uint16_t>(config.home_encoder_yaw);
    params.home_encoder_pitch = static_cast<uint16_t>(config.home_encoder_pitch);
    params.command_time_ms = static_cast<uint16_t>(config.servo_command_time_ms);
    params.home_move_time_ms = static_cast<uint16_t>(config.home_move_time_ms);

    std::lock_guard<std::mutex> lock(mutex_);
    control_params_ = params;
    return true;
}

void ServoPlanner::move_home()
{
    clear_tracking_command();
    std::lock_guard<std::mutex> lock(mutex_);
    fixed_id1_ = 1;
    fixed_pos1_ = control_params_.home_encoder_yaw;
    fixed_time1_ = control_params_.home_move_time_ms;
    fixed_id2_ = 2;
    fixed_pos2_ = control_params_.home_encoder_pitch;
    fixed_time2_ = control_params_.home_move_time_ms;
    mode_ = Mode::FIXED;
    target_initialized_ = false;
    std::cout << "ServoPlanner: 云台回 home" << std::endl;
}

void ServoPlanner::set_tracking_command(const TrackingCommand& command)
{
    std::lock_guard<std::mutex> lock(tracking_command_mutex_);
    latest_tracking_command_ = command;
    tracking_command_receive_time_ = std::chrono::steady_clock::now();
}

void ServoPlanner::clear_tracking_command()
{
    std::lock_guard<std::mutex> lock(tracking_command_mutex_);
    latest_tracking_command_ = TrackingCommand{};
    tracking_command_receive_time_ = std::chrono::steady_clock::now();
}

void ServoPlanner::hold_current_position()
{
    uint16_t yaw = 0;
    uint16_t pitch = 0;
    if (!servo_.get_cached_position_dual(1, yaw, 2, pitch))
    {
        std::lock_guard<std::mutex> lock(mutex_);
        yaw = current_encoder_yaw_;
        pitch = current_encoder_pitch_;
    }
    set_fixed_position(1, yaw, 100, 2, pitch, 100);
    set_mode(Mode::FIXED);
}

void ServoPlanner::start()
{
    if (running_.exchange(true))
    {
        return;
    }

    update_thread_should_stop_.store(false);
    update_thread_ = std::thread(&ServoPlanner::update_thread_func, this);
    std::cout << "ServoPlanner: 已启动 (100Hz更新线程)" << std::endl;
}

void ServoPlanner::stop()
{
    if (!running_.exchange(false))
    {
        return;
    }

    update_thread_should_stop_.store(true);
    if (update_thread_.joinable())
    {
        update_thread_.join();
    }
    std::cout << "ServoPlanner: 已停止" << std::endl;
}

void ServoPlanner::update_thread_func()
{
    constexpr int UPDATE_INTERVAL_US = 10000;
    auto next_update_time =
        std::chrono::steady_clock::now() + std::chrono::microseconds(UPDATE_INTERVAL_US);

    while (!update_thread_should_stop_.load())
    {
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_update_time)
        {
            update();
            next_update_time += std::chrono::microseconds(UPDATE_INTERVAL_US);
            if (next_update_time <= now)
            {
                next_update_time = now + std::chrono::microseconds(UPDATE_INTERVAL_US);
            }
        }
        std::this_thread::sleep_until(next_update_time);
    }
}

void ServoPlanner::update()
{
    if (!running_.load())
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    switch (mode_)
    {
    case Mode::FIXED:
        update_fixed();
        break;
    case Mode::IMM_PDAF_TRACKING:
        update_imm_pdaf_tracking();
        break;
    }
}

void ServoPlanner::update_fixed()
{
    servo_.update_command_dual(
        fixed_id1_, fixed_pos1_, fixed_time1_,
        fixed_id2_, fixed_pos2_, fixed_time2_);
}

void ServoPlanner::update_imm_pdaf_tracking()
{
    uint16_t current_yaw = current_encoder_yaw_;
    uint16_t current_pitch = current_encoder_pitch_;
    servo_.get_cached_position_dual(1, current_yaw, 2, current_pitch);
    current_encoder_yaw_ = current_yaw;
    current_encoder_pitch_ = current_pitch;

    const auto now = std::chrono::steady_clock::now();
    TrackingCommand command;
    double command_age_s = 0.0;
    {
        std::lock_guard<std::mutex> lock(tracking_command_mutex_);
        command = latest_tracking_command_;
        command_age_s =
            std::chrono::duration<double>(now - tracking_command_receive_time_).count();
    }

    if (!command.valid)
    {
        return;
    }

    const double stale_timeout_s =
        std::isfinite(command.stale_timeout_s) && command.stale_timeout_s > 0.0
        ? command.stale_timeout_s : 0.30;
    if (command_age_s > stale_timeout_s)
    {
        command.source = TrackingControlSource::HOME;
    }

    if (command.source == TrackingControlSource::HOLD)
    {
        return;
    }

    double target_yaw = static_cast<double>(control_params_.home_encoder_yaw);
    double target_pitch = static_cast<double>(control_params_.home_encoder_pitch);
    if (command.source == TrackingControlSource::VISUAL_SERVO
        || command.source == TrackingControlSource::PREDICTIVE_3D)
    {
        if (!std::isfinite(command.target_yaw_rad)
            || !std::isfinite(command.target_pitch_rad))
        {
            return;
        }
        constexpr double RAD_TO_DEG = 180.0 / M_PI;
        target_yaw = command.target_yaw_rad * RAD_TO_DEG / 0.0669 + 2062.8;
        target_pitch = command.target_pitch_rad * RAD_TO_DEG / -0.071 + 1936.79;
    }

    if (!target_initialized_)
    {
        command_target_yaw_ = static_cast<double>(current_yaw);
        command_target_pitch_ = static_cast<double>(current_pitch);
        target_initialized_ = true;
    }

    double dt = std::chrono::duration<double>(now - last_control_time_).count();
    last_control_time_ = now;
    dt = std::clamp(dt, 0.0, 0.05);
    const double maximum_delta = control_params_.max_encoder_rate * dt;

    command_target_yaw_ += std::clamp(
        target_yaw - command_target_yaw_, -maximum_delta, maximum_delta);
    command_target_pitch_ += std::clamp(
        target_pitch - command_target_pitch_, -maximum_delta, maximum_delta);

    command_target_yaw_ = std::clamp(
        command_target_yaw_,
        static_cast<double>(control_params_.min_encoder_yaw),
        static_cast<double>(control_params_.max_encoder_yaw));
    command_target_pitch_ = std::clamp(
        command_target_pitch_,
        static_cast<double>(control_params_.min_encoder_pitch),
        static_cast<double>(control_params_.max_encoder_pitch));

    const uint16_t applied_yaw =
        static_cast<uint16_t>(std::lround(command_target_yaw_));
    const uint16_t applied_pitch =
        static_cast<uint16_t>(std::lround(command_target_pitch_));
    servo_.update_command_dual(
        1, applied_yaw, control_params_.command_time_ms,
        2, applied_pitch, control_params_.command_time_ms);

    if (saving_.load() && command.generated_timestamp_us > 0)
    {
        const int64_t applied_timestamp_us =
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        const int64_t command_to_planner_us =
            applied_timestamp_us >= command.generated_timestamp_us
            ? applied_timestamp_us - command.generated_timestamp_us : -1;
        const int64_t frame_to_planner_us =
            command.frame_timestamp_us > 0
            && applied_timestamp_us >= command.frame_timestamp_us
            ? applied_timestamp_us - command.frame_timestamp_us : -1;
        const bool rate_limited =
            std::abs(command_target_yaw_ - target_yaw) > 0.5
            || std::abs(command_target_pitch_ - target_pitch) > 0.5;

        std::lock_guard<std::mutex> lock(save_mutex_);
        timing_buffer_.push_back({
            ++timing_sequence_,
            command.frame_timestamp_us,
            command.generated_timestamp_us,
            applied_timestamp_us,
            command_to_planner_us,
            frame_to_planner_us,
            static_cast<int>(command.source),
            static_cast<int>(command.lifecycle),
            command_age_s,
            current_yaw,
            current_pitch,
            target_yaw,
            target_pitch,
            applied_yaw,
            applied_pitch,
            control_params_.command_time_ms,
            rate_limited
        });
    }
}

void ServoPlanner::start_saving()
{
    if (saving_.load())
    {
        return;
    }

    std::string data_base_directory;
    std::string error_message;
    if (!resolve_data_base_dir(data_base_directory, error_message))
    {
        std::cerr << "ServoPlanner: 数据保存目录不可用: "
                  << error_message << std::endl;
        return;
    }

    save_directory_ = data_base_directory + "/visual_tracking";
    try
    {
        std::filesystem::create_directories(save_directory_);
        fix_permissions(save_directory_);
    }
    catch (const std::filesystem::filesystem_error& error)
    {
        std::cerr << "ServoPlanner: 无法创建保存目录: "
                  << error.what() << std::endl;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(save_mutex_);
        timing_buffer_.clear();
        timing_sequence_ = 0;
    }
    saving_.store(true);
    std::cout << "ServoPlanner: 开始保存 IMM-PDAF 控制时序到: "
              << save_directory_ << std::endl;
}

void ServoPlanner::stop_saving()
{
    size_t count = 0;
    {
        std::lock_guard<std::mutex> lock(save_mutex_);
        count = timing_buffer_.size();
    }
    if (count > 0)
    {
        save_imm_pdaf_timing_to_csv();
    }
    {
        std::lock_guard<std::mutex> lock(save_mutex_);
        timing_buffer_.clear();
    }
    saving_.store(false);
    std::cout << "ServoPlanner: 停止保存 IMM-PDAF 控制时序，共 "
              << count << " 条" << std::endl;
}

void ServoPlanner::save_imm_pdaf_timing_to_csv()
{
    std::deque<ImmPdafControlTimingRecord> records;
    {
        std::lock_guard<std::mutex> lock(save_mutex_);
        records.swap(timing_buffer_);
    }
    if (records.empty())
    {
        return;
    }

    const auto now = std::chrono::system_clock::now();
    const auto now_time = std::chrono::system_clock::to_time_t(now);
    const auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    const auto milliseconds = now_ms.time_since_epoch().count() % 1000;
    std::tm local_time;
    localtime_r(&now_time, &local_time);

    std::stringstream filename_stream;
    filename_stream << save_directory_ << "/imm_pdaf_control_timing_"
                    << std::setfill('0') << std::setw(4) << (local_time.tm_year + 1900)
                    << std::setw(2) << (local_time.tm_mon + 1)
                    << std::setw(2) << local_time.tm_mday << "_"
                    << std::setw(2) << local_time.tm_hour
                    << std::setw(2) << local_time.tm_min
                    << std::setw(2) << local_time.tm_sec << "_"
                    << std::setw(3) << milliseconds << ".csv";
    const std::string filename = filename_stream.str();

    std::ofstream output(filename);
    if (!output.is_open())
    {
        std::cerr << "ServoPlanner: 无法打开控制时序文件: "
                  << filename << std::endl;
        return;
    }

    output << std::fixed << std::setprecision(6);
    output << "sequence,frame_timestamp_us,command_generated_timestamp_us,"
           << "planner_applied_timestamp_us,command_to_planner_time_us,"
           << "frame_to_planner_time_us,command_source,lifecycle,command_age_s,"
           << "current_encoder_yaw,current_encoder_pitch,requested_encoder_yaw,"
           << "requested_encoder_pitch,applied_encoder_yaw,applied_encoder_pitch,"
           << "command_time_ms,rate_limited\n";
    for (const auto& record : records)
    {
        output << record.sequence << ',' << record.frame_timestamp_us << ','
               << record.command_generated_timestamp_us << ','
               << record.planner_applied_timestamp_us << ','
               << record.command_to_planner_time_us << ','
               << record.frame_to_planner_time_us << ','
               << record.command_source << ',' << record.lifecycle << ','
               << record.command_age_s << ','
               << record.current_encoder_yaw << ','
               << record.current_encoder_pitch << ','
               << record.requested_encoder_yaw << ','
               << record.requested_encoder_pitch << ','
               << record.applied_encoder_yaw << ','
               << record.applied_encoder_pitch << ','
               << record.command_time_ms << ','
               << (record.rate_limited ? 1 : 0) << '\n';
    }
    output.close();
    fix_permissions(filename);
}

void ServoPlanner::fix_permissions(const std::string& path)
{
    const char* sudo_uid = std::getenv("SUDO_UID");
    const char* sudo_gid = std::getenv("SUDO_GID");
    if (!sudo_uid || !sudo_gid)
    {
        return;
    }

    const uid_t uid = static_cast<uid_t>(std::stoul(sudo_uid));
    const gid_t gid = static_cast<gid_t>(std::stoul(sudo_gid));
    if (chown(path.c_str(), uid, gid) != 0)
    {
        return;
    }
    struct stat status;
    if (stat(path.c_str(), &status) == 0)
    {
        chmod(path.c_str(), S_ISDIR(status.st_mode) ? 0775 : 0664);
    }
}
