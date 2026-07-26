/**
 * @file common.h
 * @brief 云台位姿类型与角度换算工具。
 *        Gimbal pose types and angle-conversion utilities.
 */

#pragma once

#include <cstdint>
#include <cmath>

struct GimbalPose {
    double yaw;
    double pitch;
    int64_t timestamp_us;
};

struct GimbalPoseHistory {
    double yaw;
    double pitch;
    int64_t timestamp_us;
};

inline double deg_to_rad(double deg)
{
    return deg * M_PI / 180.0;
}

inline double compute_yaw_angle(double encoder_value)
{
    return 0.0669 * (encoder_value - 2062.8);
}

inline double compute_pitch_angle(double encoder_value)
{
    return -0.071 * (encoder_value - 1936.79);
}
