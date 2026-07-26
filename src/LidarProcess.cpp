/**
 * @file LidarProcess.cpp
 * @brief RS-M1 点云采集实现。
 *        RS-M1 point-cloud acquisition.
 */

#include "LidarProcess.h"

#include <chrono>
#include <cmath>
#include <sstream>
#include <utility>

LidarProcess::LidarProcess()
    : driver_ptr_(nullptr), 
      is_initialized_(false),
      m_udp_socket_fd(-1),
      m_udp_initialized(false)
{
    std::cout << "LidarProcess: 对象已创建。" << std::endl;
}

LidarProcess::~LidarProcess()
{
    stop();
}

bool LidarProcess::init()
{
    driver_ptr_ = std::make_shared<robosense::lidar::LidarDriver<robosense::lidar::PointCloud>>();

    robosense::lidar::RSDriverParam param;
    param.lidar_type = robosense::lidar::LidarType::RSM1;
    param.input_type = robosense::lidar::InputType::ONLINE_LIDAR;
    param.input_param.msop_port = 6699;
    param.input_param.difop_port = 7788;

    driver_ptr_->regPointCloudCallback(
        []() { return std::make_shared<robosense::lidar::PointCloud>(); },
        std::bind(&LidarProcess::point_cloud_callback, this, std::placeholders::_1)
    );

    if (!driver_ptr_->init(param))
    {
        std::cerr << "LidarProcess Error: 雷达驱动初始化失败！" << std::endl;
        return false;
    }

    driver_ptr_->start();
    std::cout << "LidarProcess: 雷达驱动已启动，等待数据..." << std::endl;

    m_udp_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_udp_socket_fd < 0)
    {
        std::cerr << "LidarProcess Warning: UDP socket创建失败，无法触发相机" << std::endl;
    }
    else
    {
        m_udp_initialized = true;
        std::cout << "LidarProcess: UDP触发器已初始化 (127.0.0.1:9999)" << std::endl;
    }

    is_initialized_ = true;
    return true;
}

void LidarProcess::stop()
{
    if (is_initialized_ && driver_ptr_)
    {
        driver_ptr_->stop();
        std::cout << "LidarProcess: 雷达驱动已停止。" << std::endl;
    }

    if (m_udp_initialized && m_udp_socket_fd >= 0)
    {
        close(m_udp_socket_fd);
        m_udp_socket_fd = -1;
        m_udp_initialized = false;
    }
}

void LidarProcess::set_data_callback(DataCallback callback)
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    data_callback_ = std::move(callback);
}

bool LidarProcess::get_point_cloud(PointCloudPtr& cloud)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    
    if (data_queue_.empty())
    {
        return false;
    }
    
    // 仅保留最新帧 / Keep only the latest frame.
    cloud = data_queue_.back();
    std::queue<PointCloudPtr> empty_queue;
    data_queue_.swap(empty_queue);
    return true;
}

void LidarProcess::point_cloud_callback(const PointCloudPtr& cloud)
{
    const int64_t lidar_callback_timestamp_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        if (data_queue_.size() >= MAX_QUEUE_SIZE)
        {
            data_queue_.pop();
        }
        
        data_queue_.push(cloud);
    }

    if (m_udp_initialized && m_udp_socket_fd >= 0)
    {
        const uint64_t trigger_sequence = m_camera_trigger_sequence.fetch_add(1) + 1;
        const int64_t trigger_sent_timestamp_us =
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        int64_t lidar_end_timestamp_us = 0;

        if (cloud && !cloud->points.empty() && std::isfinite(cloud->points.back().timestamp) &&
            cloud->points.back().timestamp > 0.0)
        {
            lidar_end_timestamp_us = static_cast<int64_t>(std::llround(cloud->points.back().timestamp * 1000000.0));
        }
        else if (cloud && std::isfinite(cloud->timestamp) && cloud->timestamp > 0.0)
        {
            // 空帧时间回退 / Timestamp fallback for empty frames.
            lidar_end_timestamp_us = static_cast<int64_t>(std::llround(cloud->timestamp * 1000000.0));
        }
        else
        {
            std::cerr << "LidarProcess Warning: 点云为空或时间戳无效，相机触发时间戳置为0"
                      << " (sequence=" << trigger_sequence << ")" << std::endl;
        }

        std::ostringstream trigger_stream;
        trigger_stream << "{\"schema_version\":2,\"trigger_sequence\":" << trigger_sequence
                       << ",\"lidar_end_timestamp_us\":" << lidar_end_timestamp_us
                       << ",\"lidar_callback_timestamp_us\":" << lidar_callback_timestamp_us
                       << ",\"trigger_sent_timestamp_us\":" << trigger_sent_timestamp_us << "}";
        const std::string trigger_message = trigger_stream.str();

        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(9999);
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
        const ssize_t sent_bytes = sendto(m_udp_socket_fd,
                                          trigger_message.data(),
                                          trigger_message.size(),
                                          0,
                                          reinterpret_cast<struct sockaddr*>(&addr),
                                          sizeof(addr));
        if (sent_bytes != static_cast<ssize_t>(trigger_message.size()))
        {
            std::cerr << "LidarProcess Warning: 相机触发消息发送失败"
                      << " (sequence=" << trigger_sequence << ")" << std::endl;
        }
    }
    
    DataCallback callback;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callback = data_callback_;
    }
    if (callback)
    {
        callback();
    }
}
