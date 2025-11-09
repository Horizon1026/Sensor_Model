#include "imu_preintegrate.h"
#include "slam_log_reporter.h"

namespace sensor_model {

template class ImuPreintegrateBlock<float>;
template class ImuPreintegrateBlock<double>;

// Reset all states.
template <typename Scalar>
void ImuPreintegrateBlock<Scalar>::Reset() {
    p_ij_ = TVec3<Scalar>::Zero();
    v_ij_ = TVec3<Scalar>::Zero();
    q_ij_ = TQuat<Scalar>::Identity();

    bias_accel_ = TVec3<Scalar>::Zero();
    bias_gyro_ = TVec3<Scalar>::Zero();

    jacobian_ = TMat15<Scalar>::Identity();
    covariance_ = TMat15<Scalar>::Zero();

    // Sequence is na_i, ng_i, na_j, ng_j, nwa, nwg
    noise_sigma_ = TVec18<Scalar>::Ones() * static_cast<Scalar>(1e-6);

    integrate_time_s_ = static_cast<Scalar>(0);
}

// Reset some states.
template <typename Scalar>
void ImuPreintegrateBlock<Scalar>::ResetIntegratedStates() {
    p_ij_ = TVec3<Scalar>::Zero();
    v_ij_ = TVec3<Scalar>::Zero();
    q_ij_ = TQuat<Scalar>::Identity();

    jacobian_ = TMat15<Scalar>::Identity();
    covariance_ = TMat15<Scalar>::Zero();

    integrate_time_s_ = static_cast<Scalar>(0);
}

// Print all states.
template <typename Scalar>
void ImuPreintegrateBlock<Scalar>::Information() const {
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

template <typename Scalar>
void ImuPreintegrateBlock<Scalar>::SimpleInformation() const {
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
template <typename Scalar>
bool ImuPreintegrateBlock<Scalar>::Propagate(const ImuMeasurement &measure_i, const ImuMeasurement &measure_j) {
    // Check integrate time.
    const double dt = measure_j.time_stamp_s - measure_i.time_stamp_s;
    if (dt < 0) {
        return false;
    }
    const double half_dt = 0.5 * dt;
    const double dt2 = dt * dt;
    integrate_time_s_ += dt;

    // Compute delta rotation.
    const TVec3<Scalar> mid_gyro = 0.5 * (measure_i.gyro.cast<Scalar>() + measure_j.gyro.cast<Scalar>()) - bias_gyro_;
    const TQuat<Scalar> dq(1.0, half_dt * mid_gyro.x(), half_dt * mid_gyro.y(), half_dt * mid_gyro.z());
    const TQuat<Scalar> new_q_ij = (q_ij_ * dq).normalized();

    // Compute delta position and velocity.
    const TVec3<Scalar> mid_accel = 0.5 * (q_ij_ * (measure_i.accel.cast<Scalar>() - bias_accel_) + new_q_ij * (measure_j.accel.cast<Scalar>() - bias_accel_));
    const TVec3<Scalar> new_p_ij = p_ij_ + v_ij_ * dt + 0.5 * mid_accel * dt2;
    const TVec3<Scalar> new_v_ij = v_ij_ + mid_accel * dt;

    // Compute F and V matrix.
    const TMat3<Scalar> Ri = q_ij_.matrix();
    const TMat3<Scalar> Rj = new_q_ij.matrix();
    const TMat3<Scalar> Rw = Utility::SkewSymmetricMatrix(mid_gyro);
    const TMat3<Scalar> Rai = Utility::SkewSymmetricMatrix(measure_i.accel.cast<Scalar>() - bias_accel_);
    const TMat3<Scalar> Raj = Utility::SkewSymmetricMatrix(measure_j.accel.cast<Scalar>() - bias_accel_);
    const TMat3<Scalar> I3 = TMat3<Scalar>::Identity();
    const TMat3<Scalar> dt_I3 = I3 * dt;

    TMat15<Scalar> F = TMat15<Scalar>::Zero();
    F.template block<3, 3>(ImuIndex::kPosition, ImuIndex::kPosition) = I3;
    F.template block<3, 3>(ImuIndex::kPosition, ImuIndex::kRotation) = -0.25 * Ri * Rai * dt2 - 0.25 * Rj * Raj * (I3 - Rw * dt) * dt2;
    F.template block<3, 3>(ImuIndex::kPosition, ImuIndex::kVelocity) = dt_I3;
    F.template block<3, 3>(ImuIndex::kPosition, ImuIndex::kBiasAccel) = -0.25 * (Ri + Rj) * dt2;
    F.template block<3, 3>(ImuIndex::kPosition, ImuIndex::kBiasGyro) = 0.25 * Rj * Raj * dt2 * dt;
    F.template block<3, 3>(ImuIndex::kRotation, ImuIndex::kRotation) = I3 - Rw * dt;
    F.template block<3, 3>(ImuIndex::kRotation, ImuIndex::kBiasGyro) = -dt_I3;
    F.template block<3, 3>(ImuIndex::kVelocity, ImuIndex::kRotation) = -Ri * Rai * half_dt - Rj * Raj * (I3 - Rw * dt) * half_dt;
    F.template block<3, 3>(ImuIndex::kVelocity, ImuIndex::kVelocity) = I3;
    F.template block<3, 3>(ImuIndex::kVelocity, ImuIndex::kBiasAccel) = -(Ri + Rj) * half_dt;
    F.template block<3, 3>(ImuIndex::kVelocity, ImuIndex::kBiasGyro) = 0.5 * Rj * Raj * dt2;
    F.template block<3, 3>(ImuIndex::kBiasAccel, ImuIndex::kBiasAccel) = I3;
    F.template block<3, 3>(ImuIndex::kBiasGyro, ImuIndex::kBiasGyro) = I3;

    TMat15x18<Scalar> V = TMat15x18<Scalar>::Zero();
    V.template block<3, 3>(ImuIndex::kPosition, ImuIndex::kMidValueNoiseAccel0) = 0.25 * Ri * dt2;
    V.template block<3, 3>(ImuIndex::kPosition, ImuIndex::kMidValueNoiseGyro0) = -0.25 * Rj * Raj * dt2 * half_dt;
    V.template block<3, 3>(ImuIndex::kPosition, ImuIndex::kMidValueNoiseAccel1) = 0.25 * Rj * dt2;
    V.template block<3, 3>(ImuIndex::kPosition, ImuIndex::kMidValueNoiseGyro1) = V.template block<3, 3>(ImuIndex::kPosition, ImuIndex::kMidValueNoiseGyro0);
    V.template block<3, 3>(ImuIndex::kRotation, ImuIndex::kMidValueNoiseGyro0) = 0.5 * dt_I3;
    V.template block<3, 3>(ImuIndex::kRotation, ImuIndex::kMidValueNoiseGyro1) = 0.5 * dt_I3;
    V.template block<3, 3>(ImuIndex::kVelocity, ImuIndex::kMidValueNoiseAccel0) = Ri * half_dt;
    V.template block<3, 3>(ImuIndex::kVelocity, ImuIndex::kMidValueNoiseGyro0) = -Rj * Raj * half_dt * half_dt;
    V.template block<3, 3>(ImuIndex::kVelocity, ImuIndex::kMidValueNoiseAccel1) = Rj * half_dt;
    V.template block<3, 3>(ImuIndex::kVelocity, ImuIndex::kMidValueNoiseGyro1) = V.template block<3, 3>(ImuIndex::kVelocity, ImuIndex::kMidValueNoiseGyro0);
    V.template block<3, 3>(ImuIndex::kBiasAccel, ImuIndex::kMidValueRandomWalkAccel) = dt_I3;
    V.template block<3, 3>(ImuIndex::kBiasGyro, ImuIndex::kMidValueRandomWalkGyro) = dt_I3;

    // In order to compute V * Q * V.t, decompose Q as sqrt(Q), and compute V * sqrt(Q) with noise model.
    for (uint32_t i = 0; i < V.cols(); ++i) {
        V.col(i) *= noise_sigma_(i);
    }

    // Update jacobian, covariance and preintegration results.
    const TMat15<Scalar> jacobian = F * jacobian_;
    const TMat15<Scalar> covariance = F * covariance_ * F.transpose() + V * V.transpose();

    jacobian_ = jacobian;
    covariance_ = covariance;
    p_ij_ = new_p_ij;
    q_ij_ = new_q_ij;
    v_ij_ = new_v_ij;

    return true;
}

// Correct integrate block with new bias_a and bias_g.
template <typename Scalar>
void ImuPreintegrateBlock<Scalar>::Correct(const TVec3<Scalar> &new_ba, const TVec3<Scalar> &new_bg, TVec3<Scalar> &corr_p_ij, TQuat<Scalar> &corr_q_ij,
                                           TVec3<Scalar> &corr_v_ij) {
    const TVec3<Scalar> delta_ba = new_ba - bias_accel_;
    const TVec3<Scalar> delta_bg = new_bg - bias_gyro_;

    corr_p_ij = p_ij_ + dp_dba() * delta_ba + dp_dbg() * delta_bg;
    corr_v_ij = v_ij_ + dv_dba() * delta_ba + dv_dbg() * delta_bg;

    const TVec3<Scalar> omega = dr_dbg() * delta_bg;
    corr_q_ij = q_ij_ * Utility::Exponent(omega);
    corr_q_ij.normalize();
}

// Set noise sigma vector.
template <typename Scalar>
void ImuPreintegrateBlock<Scalar>::SetImuNoiseSigma(const Scalar accel_noise, const Scalar gyro_noise, const Scalar accel_random_walk,
                                                    const Scalar gyro_random_walk) {
    noise_sigma_ << accel_noise, accel_noise, accel_noise, gyro_noise, gyro_noise, gyro_noise, accel_noise, accel_noise, accel_noise, gyro_noise, gyro_noise,
        gyro_noise, accel_random_walk, accel_random_walk, accel_random_walk, gyro_random_walk, gyro_random_walk, gyro_random_walk;
}

}  // namespace sensor_model
