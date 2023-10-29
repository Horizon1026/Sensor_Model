#include "imu_preintegrate.h"
#include "log_report.h"

namespace SENSOR_MODEL {

// Reset all states.
void ImuPreintegrateBlock::Reset() {
    p_ij_ = Vec3::Zero();
    v_ij_ = Vec3::Zero();
    q_ij_ = Quat::Identity();

    bias_accel_ = Vec3::Zero();
    bias_gyro_ = Vec3::Zero();

    jacobian_ = Mat15::Identity();
    covariance_ = Mat15::Zero();

    // Sequence is na_i, ng_i, na_j, ng_j, nwa, nwg
    noise_sigma_ = Vec18::Ones() * 1e-6f;

    integrate_time_s_ = 0.0f;
}

// Print all states.
void ImuPreintegrateBlock::Information() {
    ReportInfo("[Imu Preintegrate Block] Information:");
    ReportInfo(" - p_ij is " << LogVec(p_ij_));
    ReportInfo(" - v_ij is " << LogVec(v_ij_));
    ReportInfo(" - q_ij is " << LogQuat(q_ij_));
    ReportInfo(" - bias_accel is " << LogVec(bias_accel_));
    ReportInfo(" - bias_gyro is " << LogVec(bias_gyro_));
    ReportInfo(" - integrate_time is " << integrate_time_s_ << "s");
    ReportInfo(" - noise_sigma is " << LogVec(noise_sigma_));
    ReportInfo(" - jacobian is\n" << jacobian_);
    ReportInfo(" - covariance is\n" << covariance_);
}
void ImuPreintegrateBlock::SimpleInformation() {
    ReportInfo("[Imu Preintegrate Block] Information:");
    ReportInfo(" - p_ij is " << LogVec(p_ij_));
    ReportInfo(" - v_ij is " << LogVec(v_ij_));
    ReportInfo(" - q_ij is " << LogQuat(q_ij_));
    ReportInfo(" - bias_accel is " << LogVec(bias_accel_));
    ReportInfo(" - bias_gyro is " << LogVec(bias_gyro_));
    ReportInfo(" - integrate_time is " << integrate_time_s_ << "s");
    ReportInfo(" - noise_sigma is " << LogVec(noise_sigma_));
}

// Propagate integrate block.
bool ImuPreintegrateBlock::Propagate(const ImuMeasurement &measure_i,
                                     const ImuMeasurement &measure_j) {
    // Check integrate time.
    const float dt = measure_j.time_stamp_s - measure_i.time_stamp_s;
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
    F.block<3, 3>(ImuIndex::kPosition, ImuIndex::kPosition) = I3;
    F.block<3, 3>(ImuIndex::kPosition, ImuIndex::kVelocity) = - 0.25f * Ri * Rai * dt2 - 0.25f * Rj * Raj * (I3 - Rw * dt) * dt2;
    F.block<3, 3>(ImuIndex::kPosition, ImuIndex::kRotation) = dt_I3;
    F.block<3, 3>(ImuIndex::kPosition, ImuIndex::kBiasAccel) = - 0.25f * (Ri + Rj) * dt2;
    F.block<3, 3>(ImuIndex::kPosition, ImuIndex::kBiasGyro) = 0.25f * Rj * Raj * dt2 * dt;
    F.block<3, 3>(ImuIndex::kVelocity, ImuIndex::kVelocity) = I3 - Rw * dt;
    F.block<3, 3>(ImuIndex::kVelocity, ImuIndex::kBiasGyro) = - dt_I3;
    F.block<3, 3>(ImuIndex::kRotation, ImuIndex::kVelocity) = - Ri * Rai * half_dt - Rj * Raj * (I3 - Rw * dt) * half_dt;
    F.block<3, 3>(ImuIndex::kRotation, ImuIndex::kRotation) = I3;
    F.block<3, 3>(ImuIndex::kRotation, ImuIndex::kBiasAccel) = - (Ri + Rj) * half_dt;
    F.block<3, 3>(ImuIndex::kRotation, ImuIndex::kBiasGyro) = 0.5f * Rj * Raj * dt2;
    F.block<3, 3>(ImuIndex::kBiasAccel, ImuIndex::kBiasAccel) = I3;
    F.block<3, 3>(ImuIndex::kBiasGyro, ImuIndex::kBiasGyro) = I3;

    Mat15x18 V = Mat15x18::Zero();
    V.block<3, 3>(ImuIndex::kPosition, ImuIndex::kMidValueNoiseAccel0) = 0.25f * Ri * dt2;
    V.block<3, 3>(ImuIndex::kPosition, ImuIndex::kMidValueNoiseGyro0) = - 0.25f * Rj * Raj * dt2 * half_dt;
    V.block<3, 3>(ImuIndex::kPosition, ImuIndex::kMidValueNoiseAccel1) = 0.25f * Rj * dt2;
    V.block<3, 3>(ImuIndex::kPosition, ImuIndex::kMidValueNoiseGyro1) = V.block<3, 3>(ImuIndex::kPosition, ImuIndex::kMidValueNoiseGyro0);
    V.block<3, 3>(ImuIndex::kVelocity, ImuIndex::kMidValueNoiseGyro0) = 0.5f * dt_I3;
    V.block<3, 3>(ImuIndex::kVelocity, ImuIndex::kMidValueNoiseGyro1) = 0.5f * dt_I3;
    V.block<3, 3>(ImuIndex::kRotation, ImuIndex::kMidValueNoiseAccel0) = Ri * half_dt;
    V.block<3, 3>(ImuIndex::kRotation, ImuIndex::kMidValueNoiseGyro0) = - Rj * Raj * half_dt * half_dt;
    V.block<3, 3>(ImuIndex::kRotation, ImuIndex::kMidValueNoiseAccel1) = Rj * half_dt;
    V.block<3, 3>(ImuIndex::kRotation, ImuIndex::kMidValueNoiseGyro1) = V.block<3, 3>(ImuIndex::kRotation, ImuIndex::kMidValueNoiseGyro0);
    V.block<3, 3>(ImuIndex::kBiasAccel, ImuIndex::kMidValueRandomWalkAccel) = dt_I3;
    V.block<3, 3>(ImuIndex::kBiasGyro, ImuIndex::kMidValueRandomWalkGyro) = dt_I3;

    // In order to compute V * Q * V.t, decompose Q as sqrt(Q), and compute V * sqrt(Q) with noise model.
    for (uint32_t i = 0; i < 18; ++i) {
        V.col(i) *= noise_sigma_(i);
    }

    // Update jacobian, covariance and preintegration results.
    const Mat15 jacobian = F * jacobian_;
    const Mat15 covariance = F * covariance_ * F.transpose() + V * V.transpose();

    jacobian_ = jacobian;
    covariance_ = covariance;
    p_ij_ = new_p_ij;
    q_ij_ = new_q_ij;
    v_ij_ = new_v_ij;

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

// Set noise sigma vector.
void ImuPreintegrateBlock::SetImuNoiseSigma(const float accel_noise,
                                            const float gyro_noise,
                                            const float accel_random_walk,
                                            const float gyro_random_walk) {
    noise_sigma_ << accel_noise, accel_noise, accel_noise,
                    gyro_noise, gyro_noise, gyro_noise,
                    accel_noise, accel_noise, accel_noise,
                    gyro_noise, gyro_noise, gyro_noise,
                    accel_random_walk, accel_random_walk, accel_random_walk,
                    gyro_random_walk, gyro_random_walk, gyro_random_walk;
}

}
