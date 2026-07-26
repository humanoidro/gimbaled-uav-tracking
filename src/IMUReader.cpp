/**
 * @file IMUReader.cpp
 * @brief 串口 IMU 采集与解析实现。
 *        Serial IMU acquisition and parsing.
 */

#include "IMUReader.h"
#include "AppConfig.h"

#include <poll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cmath>

namespace Protocol
{
    constexpr uint8_t FRAME_HEAD = 0xFC;
    constexpr uint8_t FRAME_TAIL = 0xFD;
    constexpr uint8_t TYPE_IMU = 0x40;
    constexpr uint8_t TYPE_AHRS = 0x41;
    constexpr uint8_t IMU_PAYLOAD_LEN = 0x38;
    constexpr uint8_t AHRS_PAYLOAD_LEN = 0x30;
}

namespace
{
    constexpr int READ_FULLY_TIMEOUT_MS = 500;
    constexpr int READ_POLL_SLICE_MS = 50;
}

static bool read_fully(int fd, void *buf, size_t count)
{
    char *cbuf = static_cast<char *>(buf);
    size_t bytes_read = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(READ_FULLY_TIMEOUT_MS);

    while (bytes_read < count)
    {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline)
        {
            return false;
        }

        auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
        int poll_timeout_ms = READ_POLL_SLICE_MS;
        if (remaining_ms < poll_timeout_ms)
        {
            poll_timeout_ms = static_cast<int>(remaining_ms);
        }
        if (poll_timeout_ms < 1)
        {
            poll_timeout_ms = 1;
        }

        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;
        const int ready = poll(&pfd, 1, poll_timeout_ms);
        if (ready == 0)
        {
            continue;
        }
        if (ready < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            return false;
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
        {
            return false;
        }
        if (!(pfd.revents & POLLIN))
        {
            continue;
        }

        ssize_t result = read(fd, cbuf + bytes_read, count - bytes_read);
        if (result > 0)
        {
            bytes_read += result;
        }
        else if (result == 0)
        {
            continue;
        }
        else
        {
            if (errno == EINTR)
                continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                continue;
            return false;
        }
    }
    return true;
}

#pragma pack(push, 1)
struct IMUPayload
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float mag_x;
    float mag_y;
    float mag_z;
    float imu_temperature;
    float pressure;
    float pressure_temperature;
    int32_t timestamp_us;
    int32_t frame_count;
};

struct AHRSPayload
{
    float roll_speed;
    float pitch_speed;
    float heading_speed;
    float roll;
    float pitch;
    float yaw;
    float q1, q2, q3, q4;
    int32_t timestamp_us;
    int32_t frame_count;
};
#pragma pack(pop)

IMUReader::IMUReader(const std::string &port, int baud_rate, const std::string &save_prefix)
    : m_port(port), m_baud_rate(baud_rate), m_save_prefix(save_prefix)
{
    open_and_configure_port();

    m_is_running = true;
    m_read_thread = std::thread(&IMUReader::read_loop, this);
}

IMUReader::~IMUReader()
{
    m_is_running = false;
    if (m_read_thread.joinable())
    {
        m_read_thread.join();
    }
    close_port();
}

bool IMUReader::is_running() const
{
    return m_is_running;
}

void IMUReader::read_loop()
{
    uint8_t header[3];
    IMUPayload imu_payload;
    AHRSPayload payload;
    uint8_t trailer[5];

    while (m_is_running)
    {
        if (!read_fully(m_serial_fd, &header[0], 1))
        {
            continue;
        }

        if (header[0] != Protocol::FRAME_HEAD)
        {
            continue;
        }

        if (!read_fully(m_serial_fd, &header[1], 2))
        {
            continue;
        }

        uint8_t type = header[1];
        uint8_t len = header[2];

        if (type == Protocol::TYPE_IMU && len == Protocol::IMU_PAYLOAD_LEN)
        {
            if (!read_fully(m_serial_fd, trailer, 4))
            {
                continue;
            }

            if (!read_fully(m_serial_fd, &imu_payload, sizeof(imu_payload)))
            {
                continue;
            }

            if (!read_fully(m_serial_fd, trailer, 1))
            {
                continue;
            }
            m_latest_acc_x = imu_payload.acc_x;
            m_latest_acc_y = imu_payload.acc_y;
            m_latest_acc_z = imu_payload.acc_z;
        }
        else if (type == Protocol::TYPE_AHRS && len == Protocol::AHRS_PAYLOAD_LEN)
        {
            if (!read_fully(m_serial_fd, trailer, 4))
            {
                continue;
            }

            if (!read_fully(m_serial_fd, &payload, sizeof(payload)))
            {
                continue;
            }

            if (!read_fully(m_serial_fd, trailer, 1))
            {
                continue;
            }

            auto now = std::chrono::system_clock::now();
            {
                std::lock_guard<std::mutex> history_lock(m_history_mutex);

                IMUDataRecord history;
                history.roll_speed = payload.roll_speed - m_roll_speed_offset;
                history.pitch_speed = payload.pitch_speed - m_pitch_speed_offset;
                history.heading_speed = payload.heading_speed - m_heading_speed_offset;
                history.acc_x = m_latest_acc_x;
                history.acc_y = m_latest_acc_y;
                history.acc_z = m_latest_acc_z;
                history.roll = payload.roll;
                history.pitch = payload.pitch;
                history.yaw = payload.yaw;
                history.q1 = payload.q1;
                history.q2 = payload.q2;
                history.q3 = payload.q3;
                history.q4 = payload.q4;
                history.timestamp_us = payload.timestamp_us;
                history.frame_count = payload.frame_count;
                auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
                history.unix_timestamp_us = now_us;
                history.timestamp = now;

                if (m_imu_history.size() >= MAX_HISTORY_SIZE)
                {
                    m_imu_history.pop_front();
                }
                m_imu_history.push_back(history);
            }

            {
                std::lock_guard<std::mutex> callback_lock(m_callback_mutex);
                if (m_imu_data_update_callback)
                {
                    m_imu_data_update_callback();
                }
            }

            if (m_saving_flag.load())
            {
                auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();

                IMUDataRecord record;
                record.roll_speed = payload.roll_speed - m_roll_speed_offset;
                record.pitch_speed = payload.pitch_speed - m_pitch_speed_offset;
                record.heading_speed = payload.heading_speed - m_heading_speed_offset;
                record.acc_x = m_latest_acc_x;
                record.acc_y = m_latest_acc_y;
                record.acc_z = m_latest_acc_z;
                record.roll = payload.roll;
                record.pitch = payload.pitch;
                record.yaw = payload.yaw;
                record.q1 = payload.q1;
                record.q2 = payload.q2;
                record.q3 = payload.q3;
                record.q4 = payload.q4;
                record.timestamp_us = payload.timestamp_us;
                record.frame_count = payload.frame_count;
                record.unix_timestamp_us = now_us;
                record.timestamp = now;

                {
                    std::lock_guard<std::mutex> lock(m_buffer_mutex);
                    m_imu_data_buffer.push_back(record);
                }
            }
        }
        else
        {
            int total_len_to_skip = 4 + len + 1;
            std::vector<uint8_t> junk(total_len_to_skip);
            if (!read_fully(m_serial_fd, junk.data(), junk.size()))
            {
                continue;
            }
            continue;
        }
    }
}

void IMUReader::open_and_configure_port()
{
    m_serial_fd = open(m_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (m_serial_fd == -1)
    {
        throw std::runtime_error("Unable to open port " + m_port + " - " + strerror(errno));
    }

    struct termios options;
    if (tcgetattr(m_serial_fd, &options) != 0)
    {
        close(m_serial_fd);
        throw std::runtime_error("tcgetattr failed: " + std::string(strerror(errno)));
    }

    cfmakeraw(&options);

    speed_t baud;
    switch (m_baud_rate)
    {
    case 921600:
        baud = B921600;
        break;
    default:
        close(m_serial_fd);
        throw std::runtime_error("Unsupported baud rate");
    }
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    options.c_cflag &= ~CSTOPB;
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 5;

    tcflush(m_serial_fd, TCIOFLUSH);

    if (tcsetattr(m_serial_fd, TCSANOW, &options) != 0)
    {
        close(m_serial_fd);
        throw std::runtime_error("tcsetattr failed: " + std::string(strerror(errno)));
    }
}

void IMUReader::close_port()
{
    if (m_serial_fd != -1)
    {
        close(m_serial_fd);
        m_serial_fd = -1;
    }
}

void IMUReader::start_saving()
{
    std::string data_base_dir;
    std::string error_message;
    if (!resolve_data_base_dir(data_base_dir, error_message))
    {
        std::cerr << "IMUReader Error: 数据保存目录不可用: " << error_message << std::endl;
        m_saving_flag.store(false);
        return;
    }

    m_save_directory = data_base_dir + "/imu_data";

    {
        std::lock_guard<std::mutex> lock(m_buffer_mutex);
        m_imu_data_buffer.clear();
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

        std::cout << "IMUReader: 开始保存IMU数据到目录: " << m_save_directory << std::endl;
        m_saving_flag.store(true);
    }
    catch (const std::filesystem::filesystem_error& e)
    {
        std::cerr << "IMUReader Error: 无法创建保存目录: " << e.what() << std::endl;
        m_saving_flag.store(false);
    }
}

void IMUReader::stop_saving()
{
    size_t saved_count = 0;
    {
        std::lock_guard<std::mutex> lock(m_buffer_mutex);
        saved_count = m_imu_data_buffer.size();
    }

    if (saved_count > 0)
    {
        save_to_csv();
    }

    {
        std::lock_guard<std::mutex> lock(m_buffer_mutex);
        m_imu_data_buffer.clear();
    }

    m_saving_flag.store(false);

    std::cout << "IMUReader: 停止保存IMU数据，共保存 " << saved_count << " 条记录" << std::endl;
}

void IMUReader::set_imu_data_update_callback(std::function<void()> callback)
{
    std::lock_guard<std::mutex> lock(m_callback_mutex);
    m_imu_data_update_callback = callback;
}

bool IMUReader::get_latest_history(IMUDataRecord& history)
{
    std::lock_guard<std::mutex> lock(m_history_mutex);
    if (m_imu_history.empty())
    {
        return false;
    }
    history = m_imu_history.back();
    return true;
}

size_t IMUReader::get_all_history(std::vector<IMUDataRecord>& history_list)
{
    std::lock_guard<std::mutex> lock(m_history_mutex);
    history_list.clear();

    history_list.reserve(m_imu_history.size());
    for (const auto& history : m_imu_history)
    {
        history_list.push_back(history);
    }
    return history_list.size();
}

void IMUReader::save_to_csv()
{
    std::vector<IMUDataRecord> records_to_save;
    
    {
        std::lock_guard<std::mutex> lock(m_buffer_mutex);
        records_to_save.assign(m_imu_data_buffer.begin(), m_imu_data_buffer.end());
    }
    
    if (records_to_save.empty())
    {
        std::cout << "IMUReader: 没有IMU数据需要保存" << std::endl;
        return;
    }
    
    auto now = std::chrono::system_clock::now();
    auto now_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto ms = now_ms.time_since_epoch().count() % 1000;
    
    std::tm tm = *std::localtime(&now_t);
    
    std::stringstream ss;
    ss << m_save_directory << "/" << m_save_prefix << "_" 
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
        std::cerr << "IMUReader Error: 无法打开文件: " << filename << std::endl;
        return;
    }
    
    ofs << std::fixed << std::setprecision(6);
    
    ofs << "unix_timestamp_us,roll_speed,pitch_speed,heading_speed,roll,pitch,yaw,q1,q2,q3,q4,timestamp_us,frame_count,acc_x,acc_y,acc_z\n";
    
    for (const auto& record : records_to_save)
    {
        ofs << record.unix_timestamp_us << ","
           << record.roll_speed << ","
           << record.pitch_speed << ","
           << record.heading_speed << ","
           << record.roll << ","
           << record.pitch << ","
           << record.yaw << ","
           << record.q1 << ","
           << record.q2 << ","
           << record.q3 << ","
           << record.q4 << ","
           << record.timestamp_us << ","
           << record.frame_count << ","
           << record.acc_x << ","
           << record.acc_y << ","
           << record.acc_z << "\n";
    }
    
    ofs.close();
    
    fix_permissions(filename);
    
    std::cout << "IMUReader: IMU数据已保存到文件: " << filename 
              << ", 记录数: " << records_to_save.size() << std::endl;
}

void IMUReader::fix_permissions(const std::string& path)
{
    const char* sudo_uid_ptr = std::getenv("SUDO_UID");
    const char* sudo_gid_ptr = std::getenv("SUDO_GID");

    if (sudo_uid_ptr && sudo_gid_ptr)
    {
        uid_t target_uid = static_cast<uid_t>(std::stoul(sudo_uid_ptr));
        gid_t target_gid = static_cast<gid_t>(std::stoul(sudo_gid_ptr));

        if (chown(path.c_str(), target_uid, target_gid) != 0)
        {
            std::cerr << "IMUReader: 无法修复所有权 -> " << path
                      << " : " << strerror(errno) << std::endl;
            return;
        }

        struct stat st;
        if (stat(path.c_str(), &st) == 0)
        {
            if (S_ISDIR(st.st_mode))
            {
                chmod(path.c_str(), 0775);
            }
            else
            {
                chmod(path.c_str(), 0664);
            }
        }
    }
    else
    {
        struct stat st;
        if (stat(path.c_str(), &st) == 0)
        {
            if (S_ISDIR(st.st_mode))
            {
                chmod(path.c_str(), 0775);
            }
            else
            {
                chmod(path.c_str(), 0664);
            }
        }
    }
}

bool IMUReader::calibrate_angular_velocity(double stability_threshold, int max_retries)
{
    std::cout << "开始角速度校准 (最大重试: " << max_retries
              << ", 阈值: " << stability_threshold << " rad/s)..." << std::endl;

    bool history_was_insufficient = false;
    bool history_became_full = false;
    int unstable_count = 0;

    for (int attempt = 0; attempt < max_retries; ++attempt)
    {
        size_t history_size;
        {
            std::lock_guard<std::mutex> lock(m_history_mutex);
            history_size = m_imu_history.size();
        }

        if (history_size < MAX_HISTORY_SIZE)
        {
            if (!history_was_insufficient)
            {
                std::cout << "等待历史数据积累 (当前: " << history_size << "/" << MAX_HISTORY_SIZE << ")..." << std::endl;
                history_was_insufficient = true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }

        if (!history_became_full)
        {
            std::cout << "历史数据已满，开始稳定性检测..." << std::endl;
            history_became_full = true;
        }

        std::vector<IMUDataRecord> samples;
        {
            std::lock_guard<std::mutex> lock(m_history_mutex);
            samples.assign(m_imu_history.begin(), m_imu_history.end());
        }

        double sum_roll = 0, sum_pitch = 0, sum_heading = 0;
        for (const auto& sample : samples)
        {
            sum_roll += sample.roll_speed;
            sum_pitch += sample.pitch_speed;
            sum_heading += sample.heading_speed;
        }

        double avg_roll = sum_roll / samples.size();
        double avg_pitch = sum_pitch / samples.size();
        double avg_heading = sum_heading / samples.size();

        double var_roll = 0, var_pitch = 0, var_heading = 0;
        for (const auto& sample : samples)
        {
            double dr = sample.roll_speed - avg_roll;
            double dp = sample.pitch_speed - avg_pitch;
            double dh = sample.heading_speed - avg_heading;
            var_roll += dr * dr;
            var_pitch += dp * dp;
            var_heading += dh * dh;
        }
        var_roll /= samples.size();
        var_pitch /= samples.size();
        var_heading /= samples.size();

        double std_roll = std::sqrt(var_roll);
        double std_pitch = std::sqrt(var_pitch);
        double std_heading = std::sqrt(var_heading);

        if (std_roll > stability_threshold || std_pitch > stability_threshold || std_heading > stability_threshold)
        {
            ++unstable_count;
            if (unstable_count == 1 || unstable_count % 20 == 0)
            {
                std::cout << "角速度未稳定 (第 " << unstable_count << " 次): "
                          << "σ_roll=" << std_roll << " σ_pitch=" << std_pitch
                          << " σ_heading=" << std_heading << " rad/s" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        m_roll_speed_offset = avg_roll;
        m_pitch_speed_offset = avg_pitch;
        m_heading_speed_offset = avg_heading;

        std::cout << "角速度校准成功! [Roll: μ=" << m_roll_speed_offset << " σ=" << std_roll
                  << ", Pitch: μ=" << m_pitch_speed_offset << " σ=" << std_pitch
                  << ", Heading: μ=" << m_heading_speed_offset << " σ=" << std_heading << " rad/s]" << std::endl;
        return true;
    }

    std::cerr << "角速度校准失败：超过最大重试次数 (" << max_retries << ")" << std::endl;
    return false;
}
