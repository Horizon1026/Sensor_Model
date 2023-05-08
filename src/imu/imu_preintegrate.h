#ifndef _SENSOR_MODEL_IMU_PREINTEGRATE_H_
#define _SENSOR_MODEL_IMU_PREINTEGRATE_H_

#include "datatype_basic.h"
#include "math_kinematics.h"
#include "imu_state.h"

namespace SENSOR_MODEL {

/* Imu preintegrate block. */
class ImuPreintegrateBlock {

public:
    ImuPreintegrateBlock() = default;
    virtual ~ImuPreintegrateBlock() = default;

    // Propagate integrate block.
    void Propagate(const ImuMeasurement &measurement);

    // Repropagate integrate block with all imu measurements.
    void Repropagate(const std::vector<ImuMeasurement *> &measurements);

    // Correct integrate block with delta bias_a and bias_g.
    void Correct(const Vec3 &delta_ba, const Vec3 &delta_bg);

    // Reference for member variables.
    Vec3 &p_ij() { return p_ij_; }
    Quat &q_ij() { return q_ij_; }
    Vec3 &v_ij() { return v_ij_; }

    Vec3 &bias_accel() { return bias_accel_; }
    Vec3 &bias_gyro() { return bias_gyro_; }

    Mat15 &jacobian() { return jacobian_; }
    Mat15 &covariance() { return covariance_; }

    float &integrate_time_s() { return integrate_time_s_; }

    Mat3 dr_dbg() { return jacobian_.block<3, 3>(ImuStateIndex::kRotation, ImuStateIndex::kBiasGyro); }
    Mat3 dv_dba() { return jacobian_.block<3, 3>(ImuStateIndex::kVelocity, ImuStateIndex::kBiasAccel); }
    Mat3 dv_dbg() { return jacobian_.block<3, 3>(ImuStateIndex::kVelocity, ImuStateIndex::kBiasGyro); }
    Mat3 dp_dba() { return jacobian_.block<3, 3>(ImuStateIndex::kPosition, ImuStateIndex::kBiasAccel); }
    Mat3 dp_dbg() { return jacobian_.block<3, 3>(ImuStateIndex::kPosition, ImuStateIndex::kBiasGyro); }

private:
    Vec3 p_ij_ = Vec3::Zero();
    Quat q_ij_ = Quat::Identity();
    Vec3 v_ij_ = Vec3::Zero();

    Vec3 bias_accel_ = Vec3::Zero();
    Vec3 bias_gyro_ = Vec3::Zero();

    Mat15 jacobian_ = Mat15::Identity();
    Mat15 covariance_ = Mat15::Zero();

    float integrate_time_s_ = 0.0f;

};

}

#endif // end of _SENSOR_MODEL_IMU_PREINTEGRATE_H_
