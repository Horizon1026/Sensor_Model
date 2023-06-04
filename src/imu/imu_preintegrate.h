#ifndef _SENSOR_MODEL_IMU_PREINTEGRATE_H_
#define _SENSOR_MODEL_IMU_PREINTEGRATE_H_

#include "datatype_basic.h"
#include "math_kinematics.h"
#include "imu_measurement.h"
#include "imu_state.h"

namespace SENSOR_MODEL {

/* Imu preintegrate block. */
class ImuPreintegrateBlock {

public:
    ImuPreintegrateBlock() = default;
    virtual ~ImuPreintegrateBlock() = default;

    // Propagate integrate block.
    bool Propagate(const ImuMeasurement &measure_i,
                   const ImuMeasurement &measure_j);

    // Correct integrate block with new bias_a and bias_g.
    void Correct(const Vec3 &new_ba,
                 const Vec3 &new_bg,
                 Vec3 &corr_p_ij,
                 Quat &corr_q_ij,
                 Vec3 &corr_v_ij);

    // Set noise sigma vector.
    void SetImuNoiseSigma(const float accel_noise,
                          const float gyro_noise,
                          const float accel_random_walk,
                          const float gyro_random_walk);

    // Reference for member variables.
    Vec3 &p_ij() { return p_ij_; }
    Quat &q_ij() { return q_ij_; }
    Vec3 &v_ij() { return v_ij_; }

    Vec3 &bias_accel() { return bias_accel_; }
    Vec3 &bias_gyro() { return bias_gyro_; }

    Mat15 &jacobian() { return jacobian_; }
    Mat15 &covariance() { return covariance_; }

    Vec18 &noise_sigma() { return noise_sigma_; }

    float &integrate_time_s() { return integrate_time_s_; }

    inline Mat3 dr_dbg() { return jacobian_.block<3, 3>(ImuIndex::kRotation, ImuIndex::kBiasGyro); }
    inline Mat3 dv_dba() { return jacobian_.block<3, 3>(ImuIndex::kVelocity, ImuIndex::kBiasAccel); }
    inline Mat3 dv_dbg() { return jacobian_.block<3, 3>(ImuIndex::kVelocity, ImuIndex::kBiasGyro); }
    inline Mat3 dp_dba() { return jacobian_.block<3, 3>(ImuIndex::kPosition, ImuIndex::kBiasAccel); }
    inline Mat3 dp_dbg() { return jacobian_.block<3, 3>(ImuIndex::kPosition, ImuIndex::kBiasGyro); }

private:
    Vec3 p_ij_ = Vec3::Zero();
    Quat q_ij_ = Quat::Identity();
    Vec3 v_ij_ = Vec3::Zero();

    Vec3 bias_accel_ = Vec3::Zero();
    Vec3 bias_gyro_ = Vec3::Zero();

    Mat15 jacobian_ = Mat15::Identity();
    Mat15 covariance_ = Mat15::Zero();

    // Sequence is na_i, ng_i, na_j, ng_j, nwa, nwg
    Vec18 noise_sigma_ = Vec18::Ones() * 1e-6f;

    float integrate_time_s_ = 0.0f;

};

}

#endif // end of _SENSOR_MODEL_IMU_PREINTEGRATE_H_
