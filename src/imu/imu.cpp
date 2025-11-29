#include "imu.h"
#include "slam_log_reporter.h"

namespace sensor_model {

bool Imu::PropagateNominalState(const ImuMeasurement &meas_prev, const ImuMeasurement &meas_next, const ImuState &state_prev, ImuState &state_next) {
    if (meas_prev.time_stamp_s > meas_next.time_stamp_s) {
        return false;
    }

    // Propagate nominal state.
    Vec3 mid_accel_i = Vec3::Zero();
    Vec3 mid_gyro_i = Vec3::Zero();
    return PropagateNominalState(meas_prev, meas_next, state_prev, state_next, mid_accel_i, mid_gyro_i);
}

bool Imu::PropagateNominalState(const ImuMeasurement &meas_prev, const ImuMeasurement &meas_next, const ImuState &state_prev, ImuState &state_next,
                                Vec3 &mid_accel_i, Vec3 &mid_gyro_i) {
    if (meas_prev.time_stamp_s > meas_next.time_stamp_s) {
        return false;
    }

    const float dt = meas_next.time_stamp_s - meas_prev.time_stamp_s;
    const Vec3 &bias_a = state_prev.ba();
    const Vec3 &bias_g = state_prev.bg();
    const Vec3 &gravity = state_prev.g_w();

    // Propagate nominal attitude.
    mid_gyro_i = 0.5f * (meas_prev.gyro + meas_next.gyro) - bias_g;
    Quat dq = Utility::DeltaQ(mid_gyro_i * dt);
    state_next.q_wi() = (state_prev.q_wi() * dq).normalized();

    // Propagate nominal velocity.
    mid_accel_i = 0.5f * (meas_prev.accel + meas_next.accel) - bias_a;
    const Vec3 mid_accel_w = 0.5f * (state_prev.q_wi() * (meas_prev.accel - bias_a) + state_next.q_wi() * (meas_next.accel - bias_a));
    state_next.v_wi() = state_prev.v_wi() + (mid_accel_w - gravity) * dt;

    // Propagate nominal position.
    state_next.p_wi() = state_prev.p_wi() + 0.5f * (state_prev.v_wi() + state_next.v_wi()) * dt;

    return true;
}

bool Imu::PropagateNominalStateCovariance(const ImuMeasurement &meas_prev, const ImuMeasurement &meas_next, const Vec3 &mid_accel_i, const Vec3 &mid_gyro_i,
                                          const ImuState &state_prev, const Mat15 &cov_prev, Mat15 &cov_next) {
    return PropagateResidualStateCovariance(meas_prev, meas_next, mid_accel_i, mid_gyro_i, state_prev, cov_prev, cov_next);
}

bool Imu::PropagateResidualStateCovariance(const ImuMeasurement &meas_prev, const ImuMeasurement &meas_next, const Vec3 &mid_accel_i, const Vec3 &mid_gyro_i,
                                           const ImuState &state_prev, const Mat15 &cov_prev, Mat15 &cov_next) {

    if (meas_next.time_stamp_s - meas_prev.time_stamp_s < 0) {
        return false;
    }

    const float dt = meas_next.time_stamp_s - meas_prev.time_stamp_s;
    const Mat3 dt_I3 = dt * Mat3::Identity();
    const Mat3 R_wi_prev = state_prev.q_wi().matrix();

    Mat15 F = Mat15::Identity();
    F.block<3, 3>(ImuIndex::kPosition, ImuIndex::kVelocity) = dt_I3;
    F.block<3, 3>(ImuIndex::kVelocity, ImuIndex::kRotation) = -dt * R_wi_prev * Utility::SkewSymmetricMatrix(mid_accel_i);
    F.block<3, 3>(ImuIndex::kVelocity, ImuIndex::kBiasAccel) = -dt * R_wi_prev;
    F.block<3, 3>(ImuIndex::kRotation, ImuIndex::kRotation) = Mat3::Identity() - dt * Utility::SkewSymmetricMatrix(mid_gyro_i);
    F.block<3, 3>(ImuIndex::kRotation, ImuIndex::kBiasGyro) = -dt_I3;

    Mat15x12 G = Mat15x12::Zero();
    G.block<3, 3>(ImuIndex::kPosition, ImuIndex::kNoiseAccel) = -0.5f * dt * dt * R_wi_prev;
    G.block<3, 3>(ImuIndex::kVelocity, ImuIndex::kNoiseAccel) = -dt * R_wi_prev;
    G.block<3, 3>(ImuIndex::kRotation, ImuIndex::kNoiseGyro) = -dt_I3;
    G.block<3, 3>(ImuIndex::kBiasAccel, ImuIndex::kRandomWalkAccel) = dt_I3;
    G.block<3, 3>(ImuIndex::kBiasGyro, ImuIndex::kRandomWalkGyro) = dt_I3;

    if (noise_sigma_(0) != options_.kAccelNoiseSigma) {
        noise_sigma_.segment<3>(ImuIndex::kNoiseAccel).array() = options_.kAccelNoiseSigma;
        noise_sigma_.segment<3>(ImuIndex::kNoiseGyro).array() = options_.kGyroNoiseSigma;
        noise_sigma_.segment<3>(ImuIndex::kRandomWalkAccel).array() = options_.kAccelRandomWalkSigma;
        noise_sigma_.segment<3>(ImuIndex::kRandomWalkGyro).array() = options_.kGyroRandomWalkSigma;
    }

    // In order to compute G * Q * G.t, decompose Q as sqrt(Q), and compute G * sqrt(Q) with noise model.
    for (uint32_t i = 0; i < 12; ++i) {
        G.col(i).noalias() = G.col(i) * noise_sigma_(i);
    }
    cov_next = F * cov_prev * F.transpose() + G * G.transpose();

    return true;
}

}  // namespace sensor_model
