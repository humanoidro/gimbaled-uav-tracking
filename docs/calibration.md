# Sensor Calibration / 传感器标定

This document records the coordinate convention and calibration values used by
the released outdoor tracking code. Machine-readable camera values are stored
in `config/camera_intrinsic.json` and `config/extrinsic_calibration.json`.

本文记录公开室外追踪代码使用的坐标约定与标定值。相机相关的机器可读参数
位于 `config/camera_intrinsic.json` 与
`config/extrinsic_calibration.json`。

## Transform convention / 变换约定

`T_A_B` maps a point from frame `B` into frame `A`:

`T_A_B` 将点从坐标系 `B` 变换至坐标系 `A`：

```text
p_A = T_A_B · p_B
```

All transforms are 4×4 homogeneous matrices. Distances are in metres and
angles passed to rotation matrices are in radians.

所有变换均为 4×4 齐次矩阵；距离单位为米，传入旋转矩阵的角度单位为弧度。

## Coordinate frames / 坐标系

| Symbol / 符号 | Definition / 定义 |
|---|---|
| `W` | local world frame / 局部世界坐标系 |
| `V` | static platform frame, x forward, y left, z up / 静止平台坐标系，x 前、y 左、z 上 |
| `G1` | gimbal yaw joint frame / 云台偏航关节坐标系 |
| `G2` | gimbal pitch joint frame / 云台俯仰关节坐标系 |
| `L` | RS-M1 tracking LiDAR frame / RS-M1 追踪雷达坐标系 |
| `C` | camera frame / 相机坐标系 |

The platform remained static during each outdoor acquisition day. The online
tracking code therefore uses a constant `T_W_V`; in its local tracking frame
this constant is represented by the identity transform.

每个室外采集日内平台均保持静止，因此在线追踪代码使用恒定的 `T_W_V`；
在局部追踪坐标系中，该常量表示为单位变换。

## Kinematic chain / 运动学链

The LiDAR-to-world transform is:

雷达至世界坐标系的变换链为：

```text
T_W_L = T_W_V · T_V_G1(yaw) · T_G1_G2(pitch) · T_G2_L
```

The joint transforms used in `src/PointCloudProcess.cpp` are:

`src/PointCloudProcess.cpp` 中使用的关节变换为：

```text
T_V_G1(yaw) =
[ cos(yaw)  -sin(yaw)  0  0.18 ]
[ sin(yaw)   cos(yaw)  0  0    ]
[ 0          0         1  0.85 ]
[ 0          0         0  1    ]

T_G1_G2(pitch) =
[  cos(pitch)  0  sin(pitch)  0     ]
[  0           1  0           0     ]
[ -sin(pitch)  0  cos(pitch)  0.062 ]
[  0           0  0           1     ]
```

## Encoder-to-angle mapping / 编码器角度映射

The encoder mappings used by the estimator, point-cloud compensation and
controller are:

姿态估计、点云去畸变与控制器使用以下编码器映射：

```text
theta_yaw   =  0.0669 × (E_yaw   - 2062.8)   [deg]
theta_pitch = -0.0710 × (E_pitch - 1936.79)  [deg]
```

## LiDAR extrinsic / 雷达外参

`T_G2_L` is stored in `config/extrinsic_calibration.json` and constructed by
`PointCloudProcess::build_T_G2_L()`:

`T_G2_L` 保存于 `config/extrinsic_calibration.json`，并由
`PointCloudProcess::build_T_G2_L()` 构造：

```text
T_G2_L =
[ 1  0  0  0.018 ]
[ 0  1  0  0     ]
[ 0  0  1  0.072 ]
[ 0  0  0  1     ]
```

## Camera extrinsic / 相机外参

`T_G2_C` contains the ideal axis permutation followed by the calibrated
`Ry(+1.10 deg)` correction:

`T_G2_C` 包含理想轴置换以及标定得到的 `Ry(+1.10 deg)` 修正：

```text
T_G2_C =
[  0.0  -0.019197442400   0.999815712122  0.025 ]
[ -1.0   0.0              0.0             0.080 ]
[  0.0  -0.999815712122  -0.019197442400  0.058 ]
[  0.0   0.0              0.0             1.000 ]
```

The tracking pipeline projects a world-frame candidate through
`T_W_C = T_W_G2 · T_G2_C`.

追踪流水线通过 `T_W_C = T_W_G2 · T_G2_C` 将世界系候选投影至相机。

## Camera intrinsics and distortion / 相机内参与畸变

The online detector uses raw 2448×2048 images. The C++ projection applies the
following radial–tangential distortion parameters:

在线检测器使用 2448×2048 原始图像；C++ 投影使用以下径向–切向畸变参数：

```text
fx = 2292.4112472233305
fy = 2289.1280016360388
cx = 1183.4111483043127
cy = 1038.7331036789315

k1 = -0.08272229239611316
k2 =  0.13203832389665585
k3 = -0.11291916041136751
p1 =  0.0003186630393428523
p2 = -0.0032283162603578613
```

## Source of truth / 权威来源

The JSON files are the runtime source of truth for camera calibration.
`T_V_G1`, `T_G1_G2` and the encoder mappings are currently defined in the C++
source listed above. When recalibrating the hardware, update both the runtime
values and this document.

相机标定以运行时 JSON 为准；`T_V_G1`、`T_G1_G2` 与编码器映射当前定义于
上述 C++ 源码中。重新标定硬件时，应同时更新运行值与本文档。
