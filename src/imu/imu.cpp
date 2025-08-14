#include "imu.h"
#include "slam_log_reporter.h"

namespace SENSOR_MODEL {

bool Imu::PropagateNominalState(const ImuMeasurement &meas_i,
                                const ImuMeasurement &meas_j,
                                const ImuState &state_i,
                                ImuState &state_j) {
    if (meas_i.time_stamp_s > meas_j.time_stamp_s) {
        return false;
    }

    // Propagate nominal state.
    Vec3 mid_accel = Vec3::Zero();
    Vec3 mid_gyro = Vec3::Zero();
    return PropagateNominalState(meas_i, meas_j, state_i, state_j, mid_accel, mid_gyro);
}

bool Imu::PropagateNominalState(const ImuMeasurement &meas_i,
                                const ImuMeasurement &meas_j,
                                const ImuState &state_i,
                                ImuState &state_j,
                                Vec3 &mid_accel,
                                Vec3 &mid_gyro) {
    if (meas_i.time_stamp_s > meas_j.time_stamp_s) {
        return false;
    }

    const float dt = meas_j.time_stamp_s - meas_i.time_stamp_s;
    const Vec3 &bias_a = state_i.ba();
    const Vec3 &bias_g = state_i.bg();
    const Vec3 &gravity = state_i.g_w();

    // Propagate nominal attitude.
    mid_gyro = 0.5f * (meas_i.gyro + meas_j.gyro) - bias_g;
    Quat dq = Utility::DeltaQ(mid_gyro * dt);
    state_j.q_wi() = (state_i.q_wi() * dq).normalized();

    // Propagate nominal velocity.
    mid_accel = 0.5f * (meas_i.accel + meas_j.accel) - bias_a;
    const Vec3 mid_accel_w = 0.5f * (state_i.q_wi() * (meas_i.accel - bias_a) + state_j.q_wi() * (meas_j.accel - bias_a));
    state_j.v_wi() = state_i.v_wi() + (mid_accel_w - gravity) * dt;

    // Propagate nominal position.
    state_j.p_wi() = state_i.p_wi() + 0.5f * (state_i.v_wi() + state_j.v_wi()) * dt;

    return true;
}

bool Imu::PropagateNominalStateCovariance(const ImuMeasurement &meas_i,
                                          const ImuMeasurement &meas_j,
                                          const Vec3 &mid_accel,
                                          const Vec3 &mid_gyro,
                                          const ImuState &state_i,
                                          const Mat15 &cov_i,
                                          Mat15 &cov_j) {
    return PropagateResidualStateCovariance(meas_i, meas_j, mid_accel, mid_gyro, state_i, cov_i, cov_j);
}

bool Imu::PropagateResidualStateCovariance(const ImuMeasurement &meas_i,
                                           const ImuMeasurement &meas_j,
                                           const Vec3 &mid_accel,
                                           const Vec3 &mid_gyro,
                                           const ImuState &state_i,
                                           const Mat15 &cov_i,
                                           Mat15 &cov_j) {

    if (meas_j.time_stamp_s - meas_i.time_stamp_s < 0) {
        return false;
    }

    const float dt = meas_j.time_stamp_s - meas_i.time_stamp_s;
    const Mat3 dt_I3 = dt * Mat3::Identity();
    const Mat3 R_wi_i = state_i.q_wi().matrix();

    Mat15 F = Mat15::Identity();
    F.block<3, 3>(ImuIndex::kPosition, ImuIndex::kVelocity) = dt_I3;
    F.block<3, 3>(ImuIndex::kVelocity, ImuIndex::kRotation) = - dt * R_wi_i * Utility::SkewSymmetricMatrix(mid_accel);
    F.block<3, 3>(ImuIndex::kVelocity, ImuIndex::kBiasAccel) = - dt * R_wi_i;
    F.block<3, 3>(ImuIndex::kRotation, ImuIndex::kRotation) = Mat3::Identity() - dt * Utility::SkewSymmetricMatrix(mid_gyro);
    F.block<3, 3>(ImuIndex::kRotation, ImuIndex::kBiasGyro) = - dt_I3;

    Mat15x12 G = Mat15x12::Zero();
    G.block<3, 3>(ImuIndex::kPosition, ImuIndex::kNoiseAccel) = - 0.5f * dt * dt * R_wi_i;
    G.block<3, 3>(ImuIndex::kVelocity, ImuIndex::kNoiseAccel) = - dt * R_wi_i;
    G.block<3, 3>(ImuIndex::kRotation, ImuIndex::kNoiseGyro) = - dt_I3;
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
    cov_j = F * cov_i * F.transpose() + G * G.transpose();

    return true;
}

}
