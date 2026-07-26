/**
 * @file ServoController.h
 * @brief 云台舵机通信与编码器反馈接口。
 *        Gimbal servo communication and encoder feedback.
 */

#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <chrono>
#include <functional>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

struct ServoCommand {
    uint8_t id;
    uint16_t angle;
    uint16_t time;
};

struct ServoPosition {
    uint16_t position;
    bool valid;
};

struct ServoPositionHistory {
    uint16_t position;
    std::chrono::system_clock::time_point timestamp;
};

struct ServoDataRecord {
    uint16_t position1;
    uint16_t position2;
    int64_t timestamp1_us;
    int64_t timestamp2_us;
};

/**
 * @brief 串口双舵机控制与位置反馈。
 *        Serial dual-servo control and position feedback.
 */
class ServoController {
public:
    ServoController(const std::string& port_name, int baud_rate);
    ~ServoController();

    bool is_open() const;

    void set_control_servo_ids(uint8_t id1, uint8_t id2);

    void update_command_dual(uint8_t id1, uint16_t angle1, uint16_t time1,
                              uint8_t id2, uint16_t angle2, uint16_t time2);

    bool get_cached_position_dual(uint8_t id1, uint16_t& position1,
                                   uint8_t id2, uint16_t& position2);

    void start_control_thread();
    void stop_control_thread();
    bool is_control_running() const;

    size_t get_all_history(uint8_t id, std::vector<ServoPositionHistory>& history_list);

    void set_servo_position_update_callback(std::function<void()> callback);

    void start_saving();
    void stop_saving();

private:
    int m_serial_port = -1;
    std::string m_port_name;

    std::thread m_send_thread;
    std::thread m_read_thread;
    std::atomic<bool> m_send_running{false};
    std::atomic<bool> m_read_running{false};
    std::atomic<bool> m_control_should_stop{false};

    std::mutex m_serial_mutex;

    std::condition_variable m_send_cv;
    std::mutex m_send_cv_mutex;
    std::condition_variable m_send_done_cv;
    std::mutex m_send_done_mutex;

    static constexpr int GUARDED_SPLIT_SEND_WAIT_US = 1500;

    uint8_t m_servo_id1;
    uint8_t m_servo_id2;

    ServoCommand m_command1;
    ServoCommand m_command2;
    std::mutex m_command_mutex;

    ServoPosition m_position1;
    ServoPosition m_position2;
    std::mutex m_position_mutex;

    static const size_t MAX_HISTORY_SIZE = 500;
    std::deque<ServoPositionHistory> m_position_history1;
    std::deque<ServoPositionHistory> m_position_history2;
    mutable std::mutex m_history_mutex;

    std::function<void()> m_servo_position_update_callback;
    std::mutex m_callback_mutex;

    std::deque<ServoDataRecord> m_servo_data_buffer;
    std::mutex m_buffer_mutex;

    std::atomic<bool> m_saving_flag{false};
    std::string m_save_directory;



    std::atomic<uint64_t> m_command_generation{0};
    std::atomic<uint64_t> m_sent_command_generation{0};

    void send_thread_func();
    void read_thread_func();
    void wait_for_guarded_split_send_window();
    void mark_command_sent(uint64_t command_generation);

    bool get_position_with_timestamp(uint8_t id, uint16_t& position, 
                                     std::chrono::system_clock::time_point& timestamp, 
                                     int timeout_ms = 50);
    bool set_position_dual(uint8_t id1, uint16_t angle1, uint16_t time1,
                           uint8_t id2, uint16_t angle2, uint16_t time2);

    void save_to_csv();

    void fix_permissions(const std::string& path);

    bool send_packet(const std::vector<uint8_t>& data_for_checksum);

    static uint8_t calculate_checksum(const std::vector<uint8_t>& data_for_checksum);
};
