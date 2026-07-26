/**
 * @file LidarProcess.h
 * @brief RS-M1 点云采集接口。
 *        RS-M1 point-cloud acquisition interface.
 */

#pragma once

#include <iostream>
#include <memory>
#include <functional>
#include <string>
#include <iomanip>
#include <mutex>
#include <queue>
#include <atomic>
#include <cstdint>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "rs_driver/api/lidar_driver.hpp"
#include "rs_driver/msg/pcl_point_cloud_msg.hpp" 

using PointT = PointXYZIRT;
namespace robosense { namespace lidar {
    using PointCloud = PointCloudT<PointT>;
}}
using PointCloudPtr = std::shared_ptr<robosense::lidar::PointCloud>;

/**
 * @brief RoboSense 点云采集与缓存。
 *        RoboSense point-cloud acquisition and buffering.
 */
class LidarProcess
{
public:
    using DataCallback = std::function<void()>;

    LidarProcess();
    ~LidarProcess();

    LidarProcess(const LidarProcess&) = delete;
    LidarProcess& operator=(const LidarProcess&) = delete;

    bool init();

    void stop();
    
    void set_data_callback(DataCallback callback);
    
    bool get_point_cloud(PointCloudPtr& cloud);

private:
    static const size_t MAX_QUEUE_SIZE = 5;
    
    void point_cloud_callback(const PointCloudPtr& cloud);

    std::shared_ptr<robosense::lidar::LidarDriver<robosense::lidar::PointCloud>> driver_ptr_;
    bool is_initialized_;

    std::queue<PointCloudPtr> data_queue_;
    mutable std::mutex queue_mutex_;
    
    DataCallback data_callback_;
    mutable std::mutex callback_mutex_;

    int m_udp_socket_fd;
    bool m_udp_initialized;
    std::atomic<uint64_t> m_camera_trigger_sequence{0};
};
