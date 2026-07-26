/**
 * @file UdpListener.cpp
 * @brief 异步 UDP 整数指令接收实现。
 *        Asynchronous UDP integer-command listener.
 */

#include "UdpListener.h"

#include <cstring>
#include <iostream>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

UdpListener::UdpListener(int port)
    : listen_port_(port),
      running_(false)
{
}

UdpListener::~UdpListener()
{
    stop();
}

bool UdpListener::start()
{
    if (running_.exchange(true))
    {
        return false;
    }

    listen_thread_ = std::thread(&UdpListener::listen_loop, this);
    process_thread_ = std::thread(&UdpListener::process_loop, this);
    return true;
}

void UdpListener::stop()
{
    if (!running_.exchange(false))
    {
        return;
    }

    queue_condition_.notify_all();
    if (listen_thread_.joinable())
    {
        listen_thread_.join();
    }
    if (process_thread_.joinable())
    {
        process_thread_.join();
    }
}

void UdpListener::set_command_callback(CommandCallback callback)
{
    command_callback_ = std::move(callback);
}

void UdpListener::listen_loop()
{
    const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0)
    {
        std::cerr << "UdpListener: 无法创建 socket" << std::endl;
        running_.store(false);
        queue_condition_.notify_all();
        return;
    }

    int reuse_address = 1;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR,
                   &reuse_address, sizeof(reuse_address)) < 0)
    {
        std::cerr << "UdpListener: 无法设置 SO_REUSEADDR" << std::endl;
        close(socket_fd);
        running_.store(false);
        queue_condition_.notify_all();
        return;
    }

    sockaddr_in server_address;
    std::memset(&server_address, 0, sizeof(server_address));
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    server_address.sin_port = htons(listen_port_);
    if (bind(socket_fd, reinterpret_cast<sockaddr*>(&server_address),
             sizeof(server_address)) < 0)
    {
        std::cerr << "UdpListener: 无法绑定 127.0.0.1:"
                  << listen_port_ << std::endl;
        close(socket_fd);
        running_.store(false);
        queue_condition_.notify_all();
        return;
    }

    while (running_.load())
    {
        fd_set read_descriptors;
        FD_ZERO(&read_descriptors);
        FD_SET(socket_fd, &read_descriptors);

        timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;
        const int selected = select(
            socket_fd + 1, &read_descriptors, nullptr, nullptr, &timeout);
        if (selected < 0)
        {
            if (running_.load())
            {
                std::cerr << "UdpListener: select 失败" << std::endl;
            }
            break;
        }
        if (selected == 0)
        {
            continue;
        }

        char buffer[64];
        const ssize_t received = recvfrom(
            socket_fd, buffer, sizeof(buffer), 0, nullptr, nullptr);
        if (received <= 0)
        {
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            message_queue_.emplace(buffer, static_cast<size_t>(received));
        }
        queue_condition_.notify_one();
    }

    close(socket_fd);
}

void UdpListener::process_message(const std::string& message)
{
    const size_t first = message.find_first_not_of(" \t\r\n");
    const size_t last = message.find_last_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
        return;
    }
    const std::string payload = message.substr(first, last - first + 1);

    try
    {
        size_t parsed = 0;
        const int command = std::stoi(payload, &parsed);
        if (parsed != payload.size())
        {
            throw std::invalid_argument("命令包含非数字字符");
        }
        if (command_callback_)
        {
            command_callback_(command);
        }
    }
    catch (const std::exception& error)
    {
        std::cerr << "UdpListener: 无效命令: " << error.what() << std::endl;
    }
}

void UdpListener::process_loop()
{
    while (running_.load())
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_condition_.wait(lock, [this]
        {
            return !message_queue_.empty() || !running_.load();
        });

        if (!running_.load())
        {
            break;
        }

        std::string message = std::move(message_queue_.front());
        message_queue_.pop();
        lock.unlock();
        process_message(message);
    }
}
