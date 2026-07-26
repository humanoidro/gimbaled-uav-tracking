/**
 * @file ServoController.cpp
 * @brief 云台舵机通信与反馈实现。
 *        Gimbal servo communication and feedback.
 */

#include "ServoController.h"
#include "AppConfig.h"
#include <iomanip>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <ctime>
#include <unistd.h>
#include <cstring>
#include <cerrno>

ServoController::ServoController(const std::string &port_name, int baud_rate)
    : m_port_name(port_name),
      m_control_should_stop(false),
      m_servo_id1(1),
      m_servo_id2(2)
{
    m_serial_port = open(port_name.c_str(), O_RDWR | O_NOCTTY);

    if (m_serial_port < 0)
    {
        std::cerr << "错误: 无法打开串口 " << port_name << "。\n";
        return;
    }

    struct termios tty;
    if (tcgetattr(m_serial_port, &tty) != 0)
    {
        std::cerr << "错误: 获取串口属性失败 (tcgetattr)\n";
        close(m_serial_port);
        m_serial_port = -1;
        return;
    }

    speed_t baud;
    switch(baud_rate) {
        case 9600:
            baud = B9600;
            break;
        case 19200:
            baud = B19200;
            break;
        case 38400:
            baud = B38400;
            break;
        case 57600:
            baud = B57600;
            break;
        case 115200:
            baud = B115200;
            break;
        case 230400:
            baud = B230400;
            break;
        case 460800:
            baud = B460800;
            break;
        case 500000:
            baud = B500000;
            break;
        case 921600:
            baud = B921600;
            break;
        case 1000000:
            baud = B1000000;
            break;
        default:
            std::cerr << "错误: 不支持的波特率 " << baud_rate << ", 使用默认值 B115200\n";
            baud = B115200;
            break;
    }

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    cfmakeraw(&tty);
    tcflush(m_serial_port, TCIOFLUSH);

    if (tcsetattr(m_serial_port, TCSANOW, &tty) != 0)
    {
        std::cerr << "错误: 设置串口属性失败 (tcsetattr)\n";
        close(m_serial_port);
        m_serial_port = -1;
    }
    else
    {
        std::cout << "串口 " << port_name << " 已打开，波特率: " << baud_rate << std::endl;
    }

    if (!is_open())
    {
        std::cerr << "错误: 串口未打开。\n";
        return;
    }

    m_command1 = {m_servo_id1, 2048, 10};
    m_command2 = {m_servo_id2, 2048, 10};
    m_position1 = {0, false};
    m_position2 = {0, false};
}

ServoController::~ServoController()
{
    stop_control_thread();

    if (is_open())
    {
        close(m_serial_port);
    }
}

bool ServoController::is_open() const
{
    return m_serial_port >= 0;
}

bool ServoController::get_position_with_timestamp(uint8_t id, uint16_t& position, 
                                                   std::chrono::system_clock::time_point& timestamp, 
                                                   int timeout_ms)
{
    if (!is_open())
    {
        std::cerr << "错误: 串口未打开，无法发送指令。\n";
        return false;
    }

    tcflush(m_serial_port, TCIFLUSH);

    std::this_thread::sleep_for(std::chrono::microseconds(500));

    auto start_time = std::chrono::steady_clock::now();

    if (!send_packet({id, 0x04, 0x02, 0x38, 0x02}))
    {
        return false;
    }

    std::vector<uint8_t> response(8);
    size_t bytes_read = 0;
    const auto timeout = std::chrono::milliseconds(timeout_ms);

    while (bytes_read < response.size())
    {
        auto current_time = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time) > timeout)
        {
            std::cout << "警告: ID " << (int)id << " 读取超时，已读取 " << bytes_read << " 字节";
            if (bytes_read > 0) {
                std::cout << " [";
                for (size_t i = 0; i < bytes_read; i++) {
                    std::cout << " " << std::hex << std::setw(2) << std::setfill('0') 
                              << (int)response[i];
                }
                std::cout << " ]" << std::dec;
            }
            std::cout << std::endl;
            return false;
        }

        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(m_serial_port, &read_fds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 5000;

        int select_result = select(m_serial_port + 1, &read_fds, NULL, NULL, &tv);

        if (select_result > 0 && FD_ISSET(m_serial_port, &read_fds))
        {
            ssize_t n = read(m_serial_port, response.data() + bytes_read, response.size() - bytes_read);
            if (n > 0)
            {
                bytes_read += n;
            }
            else if (n < 0)
            {
                std::cout << "警告: ID " << (int)id << " 读取错误" << std::endl;
                return false;
            }
        }
        else if (select_result < 0)
        {
            std::cout << "警告: ID " << (int)id << " select 错误" << std::endl;
            return false;
        }
    }

    if (response[0] != 0xFF || response[1] != 0xF5)
    {
        std::cout << "警告: ID " << (int)id << " 响应头错误 [";
        for (int i = 0; i < 8; i++) {
            std::cout << " " << std::hex << std::setw(2) << std::setfill('0') << (int)response[i];
        }
        std::cout << " ]" << std::dec << std::endl;
        return false;
    }
    
    if (response[2] != id)
    {
        std::cout << "警告: ID " << (int)id << " 响应ID不匹配，收到ID=" 
                  << (int)response[2] << std::endl;
        return false;
    }

    uint8_t expected_checksum = ~(response[2] + response[3] + response[4] + 
                                  response[5] + response[6]);
    if (response[7] != expected_checksum)
    {
        std::cout << "警告: ID " << (int)id << " 校验和错误，期望=" 
                  << std::hex << (int)expected_checksum 
                  << " 实际=" << (int)response[7] << std::dec << std::endl;
        return false;
    }

    if (response[4] != 0x00)
    {
        std::cout << "提示: ID " << (int)id << " 舵机状态异常: 0x" 
                  << std::hex << (int)response[4] << std::dec << std::endl;
    }

    position = (static_cast<uint16_t>(response[5]) << 8) | static_cast<uint16_t>(response[6]);
    timestamp = std::chrono::system_clock::now();

    return true;
}

bool ServoController::set_position_dual(uint8_t id1, uint16_t angle1, uint16_t time1,
                                         uint8_t id2, uint16_t angle2, uint16_t time2)
{
    if (!is_open())
    {
        std::cerr << "错误: 串口未打开，无法发送指令。\n";
        return false;
    }

    std::vector<uint8_t> data = {
        0xFE,
        0x0E,
        0x83,
        0x2A,
        0x04,
        id1,
        static_cast<uint8_t>(angle1 >> 8),
        static_cast<uint8_t>(angle1 & 0xFF),
        static_cast<uint8_t>(time1 >> 8),
        static_cast<uint8_t>(time1 & 0xFF),
        id2,
        static_cast<uint8_t>(angle2 >> 8),
        static_cast<uint8_t>(angle2 & 0xFF),
        static_cast<uint8_t>(time2 >> 8),
        static_cast<uint8_t>(time2 & 0xFF)
    };

    return send_packet(data);
}

uint8_t ServoController::calculate_checksum(const std::vector<uint8_t> &data_for_checksum)
{
    uint8_t sum = 0;
    for (uint8_t byte : data_for_checksum)
    {
        sum += byte;
    }
    return ~sum;
}

void ServoController::set_control_servo_ids(uint8_t id1, uint8_t id2)
{
    m_servo_id1 = id1;
    m_servo_id2 = id2;

    std::lock_guard<std::mutex> lock(m_command_mutex);
    m_command1.id = id1;
    m_command2.id = id2;
}

void ServoController::update_command_dual(uint8_t id1, uint16_t angle1, uint16_t time1,
                                           uint8_t id2, uint16_t angle2, uint16_t time2)
{
    {
        std::lock_guard<std::mutex> lock(m_command_mutex);
        m_command1.id = id1;
        m_command1.angle = angle1;
        m_command1.time = time1;
        m_command2.id = id2;
        m_command2.angle = angle2;
        m_command2.time = time2;
        m_command_generation.fetch_add(1, std::memory_order_release);
    }
    m_send_cv.notify_one();
}

bool ServoController::get_cached_position_dual(uint8_t id1, uint16_t& position1,
                                               uint8_t id2, uint16_t& position2)
{
    std::lock_guard<std::mutex> lock(m_position_mutex);
    if ((id1 == m_servo_id1 && m_position1.valid) && (id2 == m_servo_id2 && m_position2.valid))
    {
        position1 = m_position1.position;
        position2 = m_position2.position;
        return true;
    }
    return false;
}

void ServoController::start_control_thread()
{
    m_control_should_stop.store(false);

    if (m_send_running.load() || m_read_running.load())
    {
        return;
    }
    m_send_running.store(true);
    m_read_running.store(true);
    m_send_thread = std::thread(&ServoController::send_thread_func, this);
    m_read_thread = std::thread(&ServoController::read_thread_func, this);
}

void ServoController::stop_control_thread()
{
    if (!m_send_running.load() && !m_read_running.load())
    {
        return;
    }

    m_control_should_stop.store(true);
    m_send_cv.notify_one();

    if (m_send_thread.joinable())
    {
        m_send_thread.join();
    }
    if (m_read_thread.joinable())
    {
        m_read_thread.join();
    }
    m_send_running.store(false);
    m_read_running.store(false);
}

bool ServoController::is_control_running() const
{
    return m_send_running.load() || m_read_running.load();
}

void ServoController::mark_command_sent(uint64_t command_generation)
{
    uint64_t sent_generation = m_sent_command_generation.load(std::memory_order_acquire);
    if (command_generation > sent_generation)
    {
        m_sent_command_generation.store(command_generation, std::memory_order_release);
    }
    m_send_done_cv.notify_all();
}

void ServoController::wait_for_guarded_split_send_window()
{
    if (m_command_generation.load(std::memory_order_acquire) <=
        m_sent_command_generation.load(std::memory_order_acquire))
    {
        return;
    }

    m_send_cv.notify_one();

    std::unique_lock<std::mutex> lock(m_send_done_mutex);
    m_send_done_cv.wait_for(lock, std::chrono::microseconds(GUARDED_SPLIT_SEND_WAIT_US),
                            [this]() {
                                return m_control_should_stop.load() ||
                                       m_command_generation.load(std::memory_order_acquire) <=
                                       m_sent_command_generation.load(std::memory_order_acquire);
                            });
}

void ServoController::send_thread_func()
{
    const int send_interval_us = 10000;

    auto next_send_time = std::chrono::steady_clock::now() + std::chrono::microseconds(send_interval_us);

    while (!m_control_should_stop.load())
    {
        bool notified = false;
        {
            std::unique_lock<std::mutex> cv_lock(m_send_cv_mutex);
            auto result = m_send_cv.wait_until(cv_lock, next_send_time);
            notified = (result == std::cv_status::no_timeout);
        }

        auto now = std::chrono::steady_clock::now();

        bool due = (now >= next_send_time);
        bool has_pending_command = m_command_generation.load(std::memory_order_acquire) >
                                   m_sent_command_generation.load(std::memory_order_acquire);
        bool can_early = notified && has_pending_command &&
                         (now + std::chrono::microseconds(1000) < next_send_time);

        if (due || can_early)
        {
            ServoCommand cmd1, cmd2;
            uint64_t command_generation = 0;
            {
                std::lock_guard<std::mutex> lock(m_command_mutex);
                cmd1 = m_command1;
                cmd2 = m_command2;
                command_generation = m_command_generation.load(std::memory_order_acquire);
            }

            {
                std::lock_guard<std::mutex> lock(m_serial_mutex);
                if (set_position_dual(cmd1.id, cmd1.angle, cmd1.time, cmd2.id, cmd2.angle, cmd2.time))
                {
                    mark_command_sent(command_generation);
                }
            }

            if (due)
            {
                next_send_time += std::chrono::microseconds(send_interval_us);
                if (next_send_time <= now)
                {
                    next_send_time = now + std::chrono::microseconds(send_interval_us);
                }
            }
            else
            {
                next_send_time = now + std::chrono::microseconds(send_interval_us);
            }
        }
    }
}

void ServoController::read_thread_func()
{
    while (!m_control_should_stop.load())
    {
        uint16_t pos1, pos2;
        std::chrono::system_clock::time_point timestamp1, timestamp2;

        bool read_success = false;
        {
            std::lock_guard<std::mutex> lock(m_serial_mutex);
            read_success = get_position_with_timestamp(m_servo_id1, pos1, timestamp1, 50);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(2700));

        {
            std::lock_guard<std::mutex> lock(m_serial_mutex);
            read_success = read_success && get_position_with_timestamp(m_servo_id2, pos2, timestamp2, 50);
        }

        wait_for_guarded_split_send_window();

        if (read_success)
        {
            std::lock_guard<std::mutex> lock(m_position_mutex);
            m_position1.position = pos1;
            m_position1.valid = true;
            m_position2.position = pos2;
            m_position2.valid = true;

            {
                std::lock_guard<std::mutex> history_lock(m_history_mutex);

                ServoPositionHistory history1;
                history1.position = pos1;
                history1.timestamp = timestamp1;

                if (m_position_history1.size() >= MAX_HISTORY_SIZE)
                {
                    m_position_history1.pop_front();
                }
                m_position_history1.push_back(history1);

                ServoPositionHistory history2;
                history2.position = pos2;
                history2.timestamp = timestamp2;

                if (m_position_history2.size() >= MAX_HISTORY_SIZE)
                {
                    m_position_history2.pop_front();
                }
                m_position_history2.push_back(history2);
            }

            {
                std::lock_guard<std::mutex> callback_lock(m_callback_mutex);
                if (m_servo_position_update_callback)
                {
                    m_servo_position_update_callback();
                }
            }

            if (m_saving_flag.load())
            {
                auto timestamp1_us = std::chrono::duration_cast<std::chrono::microseconds>(timestamp1.time_since_epoch()).count();
                auto timestamp2_us = std::chrono::duration_cast<std::chrono::microseconds>(timestamp2.time_since_epoch()).count();

                ServoDataRecord record;
                record.position1 = pos1;
                record.position2 = pos2;
                record.timestamp1_us = timestamp1_us;
                record.timestamp2_us = timestamp2_us;

                {
                    std::lock_guard<std::mutex> buffer_lock(m_buffer_mutex);
                    m_servo_data_buffer.push_back(record);
                }
            }
        }
        else
        {
            static int fail_count = 0;
            fail_count++;
            if (fail_count % 10 == 0)
            {
                std::cout << "警告: 读取舵机位置失败 (失败次数: " << fail_count << ")" << std::endl;
            }
        }
    }
}

void ServoController::set_servo_position_update_callback(std::function<void()> callback)
{
    std::lock_guard<std::mutex> lock(m_callback_mutex);
    m_servo_position_update_callback = callback;
}

size_t ServoController::get_all_history(uint8_t id, std::vector<ServoPositionHistory>& history_list)
{
    std::lock_guard<std::mutex> lock(m_history_mutex);
    history_list.clear();

    if (id == m_servo_id1)
    {
        history_list.reserve(m_position_history1.size());
        for (const auto& history : m_position_history1)
        {
            history_list.push_back(history);
        }
        return history_list.size();
    }
    else if (id == m_servo_id2)
    {
        history_list.reserve(m_position_history2.size());
        for (const auto& history : m_position_history2)
        {
            history_list.push_back(history);
        }
        return history_list.size();
    }
    return 0;
}

void ServoController::start_saving()
{
    std::string data_base_dir;
    std::string error_message;
    if (!resolve_data_base_dir(data_base_dir, error_message))
    {
        std::cerr << "ServoController Error: 数据保存目录不可用: " << error_message << std::endl;
        m_saving_flag.store(false);
        return;
    }

    m_save_directory = data_base_dir + "/servo_data";

    {
        std::lock_guard<std::mutex> lock(m_buffer_mutex);
        m_servo_data_buffer.clear();
    }

    try
    {
        std::filesystem::create_directories(m_save_directory);

        std::filesystem::path p(m_save_directory);
        fix_permissions(p.string());

        while (p.has_parent_path() && p != p.root_path())
        {
            p = p.parent_path();
            fix_permissions(p.string());
        }

        std::cout << "ServoController: 开始保存舵机数据到目录: " << m_save_directory << std::endl;
        m_saving_flag.store(true);
    }
    catch (const std::filesystem::filesystem_error& e)
    {
        std::cerr << "ServoController Error: 无法创建保存目录: " << e.what() << std::endl;
        m_saving_flag.store(false);
    }
}

void ServoController::stop_saving()
{
    size_t saved_count = 0;
    {
        std::lock_guard<std::mutex> lock(m_buffer_mutex);
        saved_count = m_servo_data_buffer.size();
    }

    if (saved_count > 0)
    {
        save_to_csv();
    }

    {
        std::lock_guard<std::mutex> lock(m_buffer_mutex);
        m_servo_data_buffer.clear();
    }

    m_saving_flag.store(false);

    std::cout << "ServoController: 停止保存舵机数据，共保存 " << saved_count << " 条记录" << std::endl;
}

bool ServoController::send_packet(const std::vector<uint8_t>& data_for_checksum)
{
    uint8_t checksum = calculate_checksum(data_for_checksum);

    std::vector<uint8_t> command_to_send = {0xFF, 0xFF};
    command_to_send.insert(command_to_send.end(), data_for_checksum.begin(), data_for_checksum.end());
    command_to_send.push_back(checksum);

    ssize_t bytes_written = write(m_serial_port, command_to_send.data(), command_to_send.size());

    if (bytes_written != static_cast<ssize_t>(command_to_send.size()))
    {
        std::cerr << "错误: 写入串口失败或数据未完整写入。\n";
        return false;
    }

    return true;
}

void ServoController::save_to_csv()
{
    std::vector<ServoDataRecord> records_to_save;

    {
        std::lock_guard<std::mutex> lock(m_buffer_mutex);
        records_to_save.assign(m_servo_data_buffer.begin(), m_servo_data_buffer.end());
    }

    if (records_to_save.empty())
    {
        std::cout << "ServoController: 没有舵机数据需要保存" << std::endl;
        return;
    }

    auto now = std::chrono::system_clock::now();
    auto now_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto ms = now_ms.time_since_epoch().count() % 1000;

    std::tm tm = *std::localtime(&now_t);

    std::stringstream ss;
    ss << m_save_directory << "/servo_"
       << std::setfill('0') << std::setw(4) << (tm.tm_year + 1900)
       << std::setw(2) << (tm.tm_mon + 1)
       << std::setw(2) << tm.tm_mday << "_"
       << std::setw(2) << tm.tm_hour
       << std::setw(2) << tm.tm_min
       << std::setw(2) << tm.tm_sec << "_"
       << std::setw(3) << ms << ".csv";
    std::string filename = ss.str();

    std::ofstream ofs(filename);
    if (!ofs.is_open())
    {
        std::cerr << "ServoController Error: 无法打开文件: " << filename << std::endl;
        return;
    }

    ofs << std::fixed << std::setprecision(6);

    ofs << "position1,position2,timestamp1_us,timestamp2_us\n";

    for (const auto& record : records_to_save)
    {
        ofs << record.position1 << ","
           << record.position2 << ","
           << record.timestamp1_us << ","
           << record.timestamp2_us << "\n";
    }

    ofs.close();

    fix_permissions(filename);

    std::cout << "ServoController: 舵机数据已保存到文件: " << filename
              << ", 记录数: " << records_to_save.size() << std::endl;
}

void ServoController::fix_permissions(const std::string& path)
{
    const char* sudo_uid_ptr = std::getenv("SUDO_UID");
    const char* sudo_gid_ptr = std::getenv("SUDO_GID");

    if (sudo_uid_ptr && sudo_gid_ptr)
    {
        uid_t target_uid = static_cast<uid_t>(std::stoul(sudo_uid_ptr));
        gid_t target_gid = static_cast<gid_t>(std::stoul(sudo_gid_ptr));

        if (chown(path.c_str(), target_uid, target_gid) != 0)
        {
            std::cerr << "警告: 无法修改文件权限: " << path
                      << " (" << strerror(errno) << ")" << std::endl;
        }
    }
}
