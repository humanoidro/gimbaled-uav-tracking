# 构建配置 / Build configuration.
cmake_minimum_required(VERSION 3.16)

set(CMAKE_BUILD_TYPE Release)

project(avoid CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common search segmentation)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV found: ${OpenCV_VERSION}")

add_executable(avoid
    src/main.cpp
    src/AppConfig.cpp
    src/ServoController.cpp
    src/ServoPlanner.cpp
    src/IMUReader.cpp
    src/LidarProcess.cpp
    src/PointCloudProcess.cpp
    src/UdpListener.cpp
    src/GimbalPoseEstimator.cpp
    src/IMMPDAFTracker.cpp
    src/TargetTrackingPipeline.cpp
    )

target_include_directories(avoid PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

target_include_directories(avoid SYSTEM PRIVATE
    ${PROJECT_SOURCE_DIR}/third_party/rs_driver/src
    )

target_link_libraries(avoid PRIVATE
    Threads::Threads
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )

target_compile_options(avoid PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-O3 -march=native>
)
