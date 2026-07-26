/**
 * @file main.cpp
 * @brief 实时跟踪程序入口与组件生命周期管理。
 *        Real-time tracker entry point and component lifecycle.
 */

#include <csignal>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include "LidarProcess.h"
#include "ServoController.h"
#include "ServoPlanner.h"
#include "PointCloudProcess.h"
#include "UdpListener.h"
#include "IMUReader.h"
#include "GimbalPoseEstimator.h"
#include "TrackingTypes.h"
#include "TargetTrackingPipeline.h"

std::atomic<bool> main_exit(false);

void signalHandler(int signum)
{
    std::cout << "Received signal " << signum << ", shutting down..." << std::endl;
    main_exit.store(true);
    signal(signum, SIG_DFL);
}

int main()
{
    // 0. 注册退出信号 / Register shutdown signals.
    struct sigaction sa;
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    if (sigaction(SIGINT, &sa, NULL) != 0)
    {
        std::cerr << "Failed to set up SIGINT handler" << std::endl;
    }
    if (sigaction(SIGTERM, &sa, NULL) != 0)
    {
        std::cerr << "Failed to set up SIGTERM handler" << std::endl;
    }

    try
    {
        // 1. 初始化激光雷达 / Initialize the LiDAR.
        LidarProcess lidarProcess;

        if (!lidarProcess.init())
        {
            std::cerr << "Failed to initialize lidar" << std::endl;
            return -1;
        }

        // 2. 初始化云台控制与跟踪流水线 / Initialize gimbal control and tracking.
        ServoController servoController("/dev/servo", 1000000);

        if (!servoController.is_open())
        {
            std::cerr << "Failed to initialize servo controller" << std::endl;
        }
        else
        {
            uint8_t servo_id1 = 1;
            uint8_t servo_id2 = 2;

            servoController.set_control_servo_ids(servo_id1, servo_id2);
            servoController.start_control_thread();

            std::cout << "自动控制已启动 (100Hz下发 + 160Hz采集)" << std::endl;
        }

        ServoPlanner servoPlanner(servoController);
        TargetTrackingPipeline targetTrackingPipeline;
        if (!targetTrackingPipeline.load_config())
        {
            std::cerr << "IMM-PDAF 跟踪配置或标定加载失败" << std::endl;
            return -1;
        }
        if (!servoPlanner.configure_tracking(targetTrackingPipeline.get_config()))
        {
            std::cerr << "ServoPlanner 跟踪控制配置失败" << std::endl;
            return -1;
        }
        targetTrackingPipeline.set_command_callback(
            [&servoPlanner](const TrackingCommand& command)
            {
                servoPlanner.set_tracking_command(command);
            });
        if (!targetTrackingPipeline.start())
        {
            std::cerr << "IMM-PDAF 跟踪流水线启动失败" << std::endl;
            return -1;
        }
        servoPlanner.move_home();
        servoPlanner.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 3. 初始化并校准双 IMU / Initialize and calibrate both IMUs.
        IMUReader imuReader("/dev/imu_1", 921600);

        if (!imuReader.is_running())
        {
            std::cerr << "Failed to initialize IMU" << std::endl;
        }
        else
        {
            std::cout << "IMU已初始化，串口: /dev/imu_1, 波特率: 921600" << std::endl;
        }

        if (!imuReader.calibrate_angular_velocity())
        {
            std::cerr << "IMU角速度校准失败，程序终止" << std::endl;
            return -1;
        }

        imuReader.start_saving();
        servoController.start_saving();

        IMUReader vehicleImuReader("/dev/imu_2", 921600, "imu_vehicle");

        if (!vehicleImuReader.is_running())
        {
            std::cerr << "Failed to initialize vehicle IMU" << std::endl;
        }
        else
        {
            std::cout << "车身IMU已初始化，串口: /dev/imu_2, 波特率: 921600" << std::endl;
        }

        if (!vehicleImuReader.calibrate_angular_velocity())
        {
            std::cerr << "车身IMU角速度校准失败，程序终止" << std::endl;
            return -1;
        }

        vehicleImuReader.start_saving();

        // 4. 设置初始姿态并启动云台估计 / Set the initial pose and start estimation.
        servoPlanner.set_fixed_position(1, 2048, 10, 2, 2100, 10);
        servoPlanner.set_mode(ServoPlanner::Mode::FIXED);

        GimbalPoseEstimator gimbalPoseEstimator(servoController, imuReader, vehicleImuReader);

        // 5. 启动点云处理 / Start point-cloud processing.
        PointCloudProcess pointCloudProcess(lidarProcess, servoController, &gimbalPoseEstimator);

        if (!pointCloudProcess.init())
        {
            std::cerr << "Failed to initialize point cloud process" << std::endl;
            return -1;
        }

        pointCloudProcess.set_tracking_frame_callback(
            [&targetTrackingPipeline](const TrackingCloudFrame& frame)
            {
                targetTrackingPipeline.submit_frame(frame);
            });

        pointCloudProcess.start();

        // 6. 启动运行控制接口 / Start the runtime-control interface.
        UdpListener udpListener(8888);

        if (!udpListener.start())
        {
            std::cerr << "Failed to start UDP listener" << std::endl;
        }
        else
        {
            std::cout << "UDP监听器已启动，监听端口: 8888" << std::endl;
        }

        udpListener.set_command_callback([&pointCloudProcess, &imuReader, &vehicleImuReader,
                                          &servoController, &servoPlanner, &gimbalPoseEstimator,
                                          &targetTrackingPipeline](int command)
                                         {
            if (command == 9)
            {
                imuReader.start_saving();
                vehicleImuReader.start_saving();
                servoController.start_saving();
                pointCloudProcess.set_global_saving(true);
                gimbalPoseEstimator.start_saving();
                servoPlanner.start_saving();
                targetTrackingPipeline.start_saving();
                std::cout << "开始同步保存：IMU、车身IMU、舵机、雷达点云、"
                          << "EKF姿态、IMM-PDAF" << std::endl;
            }
            else if (command == 10)
            {
                if (targetTrackingPipeline.get_mode() != TargetTrackingPipeline::Mode::STOPPED)
                {
                    targetTrackingPipeline.set_mode(TargetTrackingPipeline::Mode::STOPPED);
                    servoPlanner.move_home();
                    std::cout << "IMM-PDAF 已安全停止，云台回 home" << std::endl;
                }

                imuReader.stop_saving();
                vehicleImuReader.stop_saving();
                servoController.stop_saving();
                pointCloudProcess.set_global_saving(false);
                gimbalPoseEstimator.stop_saving();
                servoPlanner.stop_saving();
                targetTrackingPipeline.stop_saving();

                std::cout << "停止同步保存" << std::endl;
            }
            else if (command == 17)
            {
                servoPlanner.clear_tracking_command();
                servoPlanner.hold_current_position();
                targetTrackingPipeline.set_mode(TargetTrackingPipeline::Mode::ACTIVE);
                servoPlanner.set_mode(ServoPlanner::Mode::IMM_PDAF_TRACKING);
                std::cout << "IMM-PDAF ACTIVE 模式已激活" << std::endl;
            }
            else
            {
                std::cout << "未知命令: " << command << std::endl;
            } });

        // 7. 等待退出信号 / Wait for a shutdown signal.
        while (!main_exit.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // 8. 按依赖顺序停止组件 / Stop components in dependency order.
        pointCloudProcess.set_tracking_frame_callback(nullptr);
        pointCloudProcess.stop();
        targetTrackingPipeline.stop();
        lidarProcess.stop();
        servoPlanner.stop();
        if (servoController.is_control_running())
        {
            servoController.stop_control_thread();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "An exception occurred: " << e.what() << std::endl;
        return -1;
    }

    std::cout << "Exiting program gracefully..." << std::endl;

    return 0;
}
