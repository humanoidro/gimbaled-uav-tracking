# Triggered camera and YOLO detector / 触发式相机与 YOLO 检测

`detect.py` is the camera-side process used in the outdoor experiments. Each
LiDAR-frame message triggers one Hikvision camera exposure and one TensorRT
inference. The result is returned to the C++ tracker over localhost UDP.

`detect.py` 是室外实验使用的相机侧进程。每条 LiDAR 帧消息触发一次海康
相机曝光与一次 TensorRT 推理，结果通过本机 UDP 返回 C++ 追踪器。

## Requirements / 依赖

- official Hikvision MVS SDK installed under `/opt/MVS`, or
  `MVS_PYTHON_PATH` set to its Python `MvImport` directory /
  海康官方 MVS SDK，安装于 `/opt/MVS`，或通过 `MVS_PYTHON_PATH`
  指定 Python `MvImport` 目录；
- Python packages `ultralytics`, `numpy` and `opencv-python` /
  Python 包 `ultralytics`、`numpy` 与 `opencv-python`；
- NVIDIA CUDA and TensorRT compatible with the target system /
  与目标系统匹配的 NVIDIA CUDA 与 TensorRT。

Vendor SDK files are not redistributed by this repository.

本仓库不再分发厂商 SDK 文件。

## Experiment model / 实验模型

Both model artifacts used for this release are retained:

本目录保留以下两个模型文件：

- `best_20260715.pt`: trained, portable source weight /
  训练得到的可移植源权重；
- `best_20260715.engine`: exact TensorRT engine used in the 16 outdoor
  flights / 16 次室外飞行实际使用的 TensorRT engine。

TensorRT engines depend on TensorRT, CUDA and GPU compatibility. Use the
provided engine to reproduce the original deployment environment, or regenerate
it from the PT weight for another compatible NVIDIA system.

TensorRT engine 依赖具体的 TensorRT、CUDA 与 GPU 兼容性。复现实验部署
环境时可直接使用随仓库提供的 engine；在其他 NVIDIA 系统上应由 PT
重新导出。

Both artifacts were produced with Ultralytics YOLO and retain its AGPL-3.0
license metadata. A separate Ultralytics commercial license, if applicable,
supersedes the open-source terms only within its agreed scope.

两个模型产物均由 Ultralytics YOLO 生成，并保留其 AGPL-3.0 许可元数据；
若另有 Ultralytics 商业许可，则仅在约定范围内以相应商业条款为准。

Generate an FP16 static-shape TensorRT engine on the target computer:

在目标计算机上生成 FP16 静态形状 TensorRT engine：

```bash
python scripts/yolo/convert_to_engine.py
```

The export uses image size 2464 and writes
`scripts/yolo/best_20260715.engine`.

导出分辨率为 2464，生成
`scripts/yolo/best_20260715.engine`。

Alternative weights and CUDA devices can be selected with `--help`.

可通过 `--help` 指定其他权重路径或 CUDA 设备。

## Run / 运行

```bash
source /opt/MVS/set_env_path.sh
python scripts/yolo/detect.py
```

By default the process loads the generated engine and the JSON files under
`config/`, saves raw camera frames below the configured `data_base_dir`, and
keeps visualization disabled.

默认加载生成的 engine 与 `config/` 下的 JSON，将原始图像保存到
`data_base_dir`，并关闭实时显示。

Run `python scripts/yolo/detect.py --help` to override the engine,
configuration, device and image paths.

使用 `python scripts/yolo/detect.py --help` 可指定 engine、配置、设备与
图像目录。

## Synchronization protocol / 同步协议

The C++ process sends one JSON object to `127.0.0.1:9999`:

C++ 进程向 `127.0.0.1:9999` 发送：

```json
{"schema_version":2,"trigger_sequence":123,"lidar_end_timestamp_us":1783650000123456}
```

After acquisition and inference, `detect.py` sends a JSON response to
`127.0.0.1:11000`. Empty detection is represented explicitly by `"d":[]`.
The response retains `trigger_sequence` and `lidar_end_timestamp_us` for exact
LiDAR–camera frame matching.

采集与推理完成后，`detect.py` 向 `127.0.0.1:11000` 返回 JSON；无目标时
明确使用 `"d":[]`。响应保留 `trigger_sequence` 与
`lidar_end_timestamp_us`，用于 LiDAR–相机帧精确匹配。

Only one trigger may be in flight. A trigger received while busy returns
`camera_busy` rather than entering a backlog. A missing image callback returns
`frame_timeout` and restarts the capture stream.

系统只允许一个在途触发。忙碌时不积压消息，而是返回 `camera_busy`；
图像回调超时则返回 `frame_timeout` 并重启取流。

Images are written asynchronously to:

图像异步保存至：

```text
<data_base_dir>/camera_images/YYYYMMDD_HHMMSS/
```

Stop with `Ctrl+C` or `SIGINT` so the image queue is flushed before exit.

请使用 `Ctrl+C` 或 `SIGINT` 正常停止，使退出前完成图像队列写盘。
