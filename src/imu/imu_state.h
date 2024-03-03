#ifndef _SENSOR_MODEL_IMU_STATE_H_
#define _SENSOR_MODEL_IMU_STATE_H_

#include "datatype_basic.h"

namespace SENSOR_MODEL {

/* Indice of imu state. */
enum ImuIndex : uint8_t {
    kPosition = 0,
    kRotation = 3,
    kVelocity = 6,
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

/* Imu state with size of 15 or 18. */
class ImuState {

public:
    ImuState() = default;
    ImuState(const Vec3 &p_wi, const Quat &q_wi, const Vec3 &v_wi, const Vec3 &ba, const Vec3 &bg, const Vec3 &g_w, const float t) :
        p_wi_(p_wi), q_wi_(q_wi), v_wi_(v_wi), ba_(ba), bg_(bg), g_w_(g_w), time_stamp_s_(t) {}
    virtual ~ImuState() = default;

    // Reference for Member Variables.
    Vec3 &p_wi() { return p_wi_; }
    Quat &q_wi() { return q_wi_; }
    Vec3 &v_wi() { return v_wi_; }
    Vec3 &ba() { return ba_; }
    Vec3 &bg() { return bg_; }
    Vec3 &g_w() { return g_w_; }
    float &time_stamp_s() { return time_stamp_s_; }
    Mat18 &covariance() { return cov_matrix_; }
    Mat18 &sqrt_cov() { return cov_matrix_; }

    // Const Reference for Member Variables.
    const Vec3 &p_wi() const { return p_wi_; }
    const Quat &q_wi() const { return q_wi_; }
    const Vec3 &v_wi() const { return v_wi_; }
    const Vec3 &ba() const { return ba_; }
    const Vec3 &bg() const { return bg_; }
    const Vec3 &g_w() const { return g_w_; }
    const float &time_stamp_s() const { return time_stamp_s_; }
    const Mat18 &covariance() const { return cov_matrix_; }
    const Mat18 &sqrt_cov() const { return cov_matrix_; }

private:
    Vec3 p_wi_ = Vec3::Zero();
    Quat q_wi_ = Quat::Identity();
    Vec3 v_wi_ = Vec3::Zero();
    Vec3 ba_ = Vec3::Zero();
    Vec3 bg_ = Vec3::Zero();
    Vec3 g_w_ = Vec3(0, 0, 9.8f);

    float time_stamp_s_ = 0.0f;

    // Shared by covariance and sqrt_covariance.
    Mat18 cov_matrix_ = Mat18::Zero();

};

}

#endif // end of _SENSOR_MODEL_IMU_STATE_H_
