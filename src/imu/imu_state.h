#ifndef _SENSOR_MODEL_IMU_STATE_H_
#define _SENSOR_MODEL_IMU_STATE_H_

#include "datatype_basic.h"
#include "math_kinematics.h"

namespace SENSOR_MODEL {

/* Indice of imu state. */
enum ImuIndex : uint8_t {
    kPosition = 0,
    kVelocity = 3,
    kRotation = 6,
    kBiasAccel = 9,
    kBiasGyro = 12,
    kGravity = 15,

    kNoiseAccel = 0,
    kNoiseGyro = 3,
    kRandomWalkAccel = 6,
    kRandomWalkGyro = 9,

    kMidValueNoiseAccel0 = 0,
    kMidValueNoiseGyro0 = 3,
    kMidValueNoiseAccel1 = 6,
    kMidValueNoiseGyro1 = 9,
    kMidValueRandomWalkAccel = 12,
    kMidValueRandomWalkGyro = 15,
};

/* Imu state with size 15 or 18. */
class ImuState {

public:
    ImuState() = default;
    ImuState(const Vec3 &p_wi, const Quat &q_wi, const Vec3 &v_wi, const Vec3 &ba, const Vec3 &bg, const Vec3 &g_w, const float t) :
        p_wi(p_wi), q_wi(q_wi), v_wi(v_wi), ba(ba), bg(bg), g_w(g_w), time_stamp_s(t) {}
    virtual ~ImuState() = default;

public:
    Vec3 p_wi = Vec3::Zero();
    Quat q_wi = Quat::Identity();
    Vec3 v_wi = Vec3::Zero();
    Vec3 ba = Vec3::Zero();
    Vec3 bg = Vec3::Zero();
    Vec3 g_w = Vec3(0, 0, 9.8f);

    float time_stamp_s = 0.0f;

};

}

#endif // end of _SENSOR_MODEL_IMU_STATE_H_
