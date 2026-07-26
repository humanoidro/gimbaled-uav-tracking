/**
 * @file UdpListener.h
 * @brief 异步 UDP 整数指令接收接口。
 *        Asynchronous UDP integer-command listener.
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

class UdpListener
{
public:
    using CommandCallback = std::function<void(int)>;

    explicit UdpListener(int port = 8888);
    ~UdpListener();

    UdpListener(const UdpListener&) = delete;
    UdpListener& operator=(const UdpListener&) = delete;

    bool start();
    void stop();
    void set_command_callback(CommandCallback callback);

private:
    void listen_loop();
    void process_loop();
    void process_message(const std::string& message);

    int listen_port_;
    std::atomic<bool> running_;
    std::thread listen_thread_;
    std::thread process_thread_;
    std::queue<std::string> message_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_condition_;
    CommandCallback command_callback_;
};
