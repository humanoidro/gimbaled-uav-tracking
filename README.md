# Gimbaled LiDAR–Camera UAV Tracking / 云台激光雷达–相机无人机追踪

Real-time gimbaled LiDAR–camera fusion for UAV detection, localization,
tracking, and two-axis gimbal control.

面向无人机检测、定位、跟踪与两轴云台控制的实时云台激光雷达–相机融合系统。

This repository contains the code and model used in all 16 outdoor flight
experiments:

本仓库保留 16 次室外飞行实验实际使用的代码与模型：

- RS-M1 point-cloud acquisition and point-level gimbal-motion compensation /
  RS-M1 点云采集与云台运动点级去畸变；
- gimbal attitude estimation from two IMUs and joint encoders /
  基于双 IMU 与关节编码器的云台姿态估计；
- synchronized camera triggering and TensorRT YOLO detection /
  相机同步触发与 TensorRT YOLO 检测；
- LiDAR–camera candidate generation and IMM-PDAF tracking /
  激光雷达–相机候选生成与 IMM-PDAF 跟踪；
- predictive two-axis gimbal control and synchronized data logging /
  两轴云台预测控制与同步数据记录。

Source repository / 源码仓库：
<https://github.com/humanoidro/gimbaled-uav-tracking>

The accompanying 16-flight dataset is released separately. Its `README.md`
defines the file formats, timestamps and evaluation protocol.

配套的 16 次飞行数据集单独发布；数据集中的 `README.md` 给出了文件格式、
时间戳与评估协议。

## Repository layout / 仓库结构

```text
config/                 Runtime parameters and sensor calibration
                        运行参数与传感器标定
docs/                   Coordinate frames and calibration reference
                        坐标系与标定说明
include/, src/          C++17 real-time system
                        C++17 实时系统
scripts/yolo/           Camera process, experiment model and TensorRT export
                        相机进程、实验模型与 TensorRT 导出
third_party/rs_driver/  Minimal RoboSense driver source required by RS-M1
                        RS-M1 所需的最小 RoboSense 驱动源码
```

Generated build files, captured data, intermediate ONNX files and vendor camera
SDK binaries are intentionally excluded. The exact TensorRT engine used in the
outdoor experiments is retained together with its source PT weight.

构建产物、采集数据、中间 ONNX 文件与相机厂商 SDK 二进制不纳入仓库；
室外实验实际使用的 TensorRT engine 与其源 PT 权重同时保留。

## Hardware interfaces / 硬件接口

The experiment program expects the following interfaces:

实验程序使用以下接口：

- RoboSense RS-M1 UDP `6699` (MSOP) and `7788` (DIFOP) /
  RoboSense RS-M1 UDP `6699`（MSOP）与 `7788`（DIFOP）；
- two-axis servo controller `/dev/servo` / 两轴舵机控制器 `/dev/servo`；
- gimbal IMU `/dev/imu_1` / 云台 IMU `/dev/imu_1`；
- platform IMU `/dev/imu_2` / 平台 IMU `/dev/imu_2`；
- Hikvision MVS camera accessed by the Python process /
  由 Python 进程访问的海康 MVS 相机。

Localhost UDP ports / 本机 UDP 端口：

- `9999`: LiDAR-frame trigger, C++ → camera /
  LiDAR 帧触发，C++ → 相机进程；
- `11000`: timestamped detection, camera → C++ /
  带时间戳的检测结果，相机进程 → C++；
- `8888`: experiment control command / 实验控制命令。

Device aliases and access permissions must be configured on the target
computer. No machine-specific udev rules or private network addresses are
stored in this repository.

设备别名与访问权限需由用户在目标计算机上配置；本仓库不保存机器相关的
udev 规则或私有网络地址。

## Dependencies / 依赖

C++ / C++ 部分：

- Linux, CMake 3.16+, and a C++17 compiler /
  Linux、CMake 3.16+ 与 C++17 编译器；
- PCL 1.8+ and OpenCV / PCL 1.8+ 与 OpenCV。

The required `rs_driver` source is included under `third_party/rs_driver` and
remains under its original BSD-3-Clause license.

必要的 `rs_driver` 源码位于 `third_party/rs_driver`，仍适用其原始
BSD-3-Clause 许可证。

Python camera process / Python 相机进程：

- official Hikvision MVS SDK / 海康官方 MVS SDK；
- Ultralytics, TensorRT, NumPy and OpenCV /
  Ultralytics、TensorRT、NumPy 与 OpenCV；
- NVIDIA CUDA-capable system for TensorRT inference /
  支持 CUDA 的 NVIDIA 平台。

## Build / 构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Run from the repository root so that the default `config/` directory is found:

从仓库根目录运行，以读取默认 `config/`：

```bash
./build/avoid
```

Set `GIMBALED_TRACKING_CONFIG_DIR` when configuration files are stored in
another directory.

若配置文件位于其他目录，可设置 `GIMBALED_TRACKING_CONFIG_DIR`。

## Camera and YOLO / 相机与 YOLO

The repository retains both `scripts/yolo/best_20260715.pt` and the exact
`best_20260715.engine` used in the outdoor experiments. The engine is tied to
the TensorRT/CUDA/GPU environment; regenerate it from the PT weight when the
provided engine is incompatible:

仓库同时保留 `scripts/yolo/best_20260715.pt` 与室外实验实际使用的
`best_20260715.engine`。engine 与 TensorRT/CUDA/GPU 环境相关；若目标
机器不兼容，请由 PT 权重重新导出：

```bash
python scripts/yolo/convert_to_engine.py
```

Start the camera and detector after installing the official MVS SDK:

安装官方 MVS SDK 后启动相机与检测进程：

```bash
python scripts/yolo/detect.py
```

Both scripts provide `--help` for overriding model, device, configuration and
data paths. See [`scripts/yolo/README.md`](scripts/yolo/README.md) for the
timestamped UDP protocol.

两个脚本均可通过 `--help` 指定模型、设备、配置和数据路径；带时间戳的 UDP
协议见 [`scripts/yolo/README.md`](scripts/yolo/README.md)。

## Configuration and calibration / 配置与标定

- `config/tracker.json`: detection, candidate, IMM, PDAF, lifecycle and control
  parameters / 检测、候选、IMM、PDAF、生命周期与控制参数；
- `config/camera_intrinsic.json`: camera intrinsic and distortion parameters /
  相机内参与畸变参数；
- `config/extrinsic_calibration.json`: `T_G2_L` and `T_G2_C` /
  `T_G2_L` 与 `T_G2_C` 外参；
- `config/app_config.json`: output data directory / 输出数据目录。

Coordinate definitions, the kinematic chain, encoder mapping and complete
matrix values are documented in
[`docs/calibration.md`](docs/calibration.md).

坐标系、运动学链、编码器映射及完整矩阵见
[`docs/calibration.md`](docs/calibration.md)。

Relative `data_base_dir` values are resolved from the repository root.
Destinations below `/media`, `/mnt` or `/run/media` must be real mounted
filesystems.

相对 `data_base_dir` 以仓库根目录解析；位于 `/media`、`/mnt` 或
`/run/media` 下的目录必须处于真实挂载的文件系统中。

## License / 许可证

Project-authored source currently carries the MIT License. Bundled third-party
source and model artifacts remain subject to their applicable upstream terms;
see `third_party/rs_driver/LICENSE`. The YOLO model artifacts embed
Ultralytics AGPL-3.0 metadata and are not relicensed by the top-level MIT
License.

项目自有源码当前采用 MIT 许可证；第三方源码与模型产物仍适用各自的上游
许可，详见 `third_party/rs_driver/LICENSE`。YOLO 模型产物嵌有
Ultralytics AGPL-3.0 元数据，不因顶层 MIT 许可证而被重新许可。
