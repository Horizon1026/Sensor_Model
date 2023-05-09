#include "imu_preintegrate.h"

namespace SENSOR_MODEL {

// Propagate integrate block.
bool ImuPreintegrateBlock::Propagate(const ImuMeasurement &measure_i,
                                     const ImuMeasurement &measure_j) {
    // Check integrate time.
    const float dt = measure_j.time_stamp - measure_i.time_stamp;
    if (dt < 0) {
        return false;
    }
    const float half_dt = 0.5f * dt;
    const float dt2 = dt * dt;
    integrate_time_s_ += dt;

    // Compute delta rotation.
    const Vec3 mid_gyro = 0.5f * (measure_i.gyro + measure_j.gyro) - bias_gyro_;
    const Quat dq(1.0f, half_dt * mid_gyro.x(), half_dt * mid_gyro.y(), half_dt * mid_gyro.z());
    const Quat new_q_ij = (q_ij_ * dq).normalized();

    // Compute delta position and velocity.
    const Vec3 mid_accel = 0.5f * (q_ij_ * (measure_i.accel - bias_accel_)
        + new_q_ij * (measure_j.accel - bias_accel_));
    const Vec3 new_p_ij = p_ij_ + v_ij_ * dt + 0.5f * mid_accel * dt2;
    const Vec3 new_v_ij = v_ij_ + mid_accel * dt;

    // Compute F and V matrix.
    const Mat3 Ri = q_ij_.matrix();
    const Mat3 Rj = new_q_ij.matrix();
    const Mat3 Rw = Utility::SkewSymmetricMatrix(mid_gyro);
    const Mat3 Rai = Utility::SkewSymmetricMatrix(measure_i.accel - bias_accel_);
    const Mat3 Raj = Utility::SkewSymmetricMatrix(measure_j.accel - bias_accel_);
    const Mat3 I3 = Mat3::Identity();
    const Mat3 dt_I3 = I3 * dt;

    Mat15 F = Mat15::Zero();
    F.block<3, 3>(ImuStateIndex::kPosition, ImuStateIndex::kPosition) = I3;
    F.block<3, 3>(ImuStateIndex::kPosition, ImuStateIndex::kVelocity) = - 0.25f * Ri * Rai * dt2 - 0.25f * Rj * Raj * (I3 - Rw * dt) * dt2;
    F.block<3, 3>(ImuStateIndex::kPosition, ImuStateIndex::kRotation) = dt_I3;
    F.block<3, 3>(ImuStateIndex::kPosition, ImuStateIndex::kBiasAccel) = - 0.25f * (Ri + Rj) * dt2;
    F.block<3, 3>(ImuStateIndex::kPosition, ImuStateIndex::kBiasGyro) = 0.25f * Rj * Raj * dt2 * dt;
    F.block<3, 3>(ImuStateIndex::kVelocity, ImuStateIndex::kVelocity) = I3 - Rw * dt;
    F.block<3, 3>(ImuStateIndex::kVelocity, ImuStateIndex::kBiasGyro) = - dt_I3;
    F.block<3, 3>(ImuStateIndex::kRotation, ImuStateIndex::kVelocity) = - Ri * Rai * half_dt - Rj * Raj * (I3 - Rw * dt) * half_dt;
    F.block<3, 3>(ImuStateIndex::kRotation, ImuStateIndex::kRotation) = I3;
    F.block<3, 3>(ImuStateIndex::kRotation, ImuStateIndex::kBiasAccel) = - (Ri + Rj) * half_dt;
    F.block<3, 3>(ImuStateIndex::kRotation, ImuStateIndex::kBiasGyro) = 0.5f * Rj * Raj * dt2;
    F.block<3, 3>(ImuStateIndex::kBiasAccel, ImuStateIndex::kBiasAccel) = I3;
    F.block<3, 3>(ImuStateIndex::kBiasGyro, ImuStateIndex::kBiasGyro) = I3;

    Mat15x18 V = Mat15x18::Zero();
    V.block<3, 3>(0, 0)   = 0.25f * Ri * dt2;
    V.block<3, 3>(0, 3)   = - 0.25f * Rj * Raj  * dt2 * half_dt;
    V.block<3, 3>(0, 6)   = 0.25f * Rj * dt2;
    V.block<3, 3>(0, 9)   = V.block<3, 3>(0, 3);
    V.block<3, 3>(3, 3)   = 0.5f * dt_I3;
    V.block<3, 3>(3, 9)   = 0.5f * dt_I3;
    V.block<3, 3>(6, 0)   = Ri * half_dt;
    V.block<3, 3>(6, 3)   = - Rj * Raj * half_dt * half_dt;
    V.block<3, 3>(6, 6)   = Rj * half_dt;
    V.block<3, 3>(6, 9)   = V.block<3, 3>(6, 3);
    V.block<3, 3>(9, 12)  = dt_I3;
    V.block<3, 3>(12, 15) = dt_I3;

    // Update jacobian, covariance and preintegration results.
    jacobian_ = F * jacobian_;
    covariance_ = F * covariance_ * F.transpose();  // + V * Q * V.t
    p_ij_ = new_p_ij;
    q_ij_ = new_q_ij;
    v_ij_ = new_v_ij;

    return true;
}

// Repropagate integrate block with all imu measurements.
bool ImuPreintegrateBlock::Repropagate(const std::vector<ImuMeasurement *> &measurements,
                                       const Vec3 &bias_accel,
                                       const Vec3 &bias_gyro) {

    return true;
}

// Correct integrate block with new bias_a and bias_g.
void ImuPreintegrateBlock::Correct(const Vec3 &new_ba,
                                   const Vec3 &new_bg,
                                   Vec3 &corr_p_ij,
                                   Quat &corr_q_ij,
                                   Vec3 &corr_v_ij) {
    const Vec3 delta_ba = new_ba - bias_accel_;
    const Vec3 delta_bg = new_bg - bias_gyro_;

    corr_p_ij = p_ij_ + dp_dba() * delta_ba + dp_dbg() * delta_bg;
    corr_v_ij = v_ij_ + dv_dba() * delta_ba + dv_dbg() * delta_bg;

    const Vec3 temp = dr_dbg() * delta_bg;
    corr_q_ij = q_ij_ * Utility::Exponent(temp).normalized();
    corr_q_ij.normalize();
}

}
