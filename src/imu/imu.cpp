#include "imu.h"

namespace SENSOR_MODEL {

bool Imu::PropagateNominalState(const ImuMeasurement &meas_i,
                                const ImuMeasurement &meas_j,
                                const ImuState &state_i,
                                ImuState &state_j) {
    if (meas_i.time_stamp > meas_j.time_stamp) {
        return false;
    }

    const float dt = meas_j.time_stamp - meas_i.time_stamp;

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
    if (meas_i.time_stamp > meas_j.time_stamp) {
        return false;
    }

    const float dt = meas_j.time_stamp - meas_i.time_stamp;
    const Vec3 &bias_a = state_i.ba;
    const Vec3 &bias_g = state_i.bg;
    const Vec3 &gravity = state_i.g_w;

    // Propagate nominal attitude.
    mid_gyro = 0.5f * (meas_i.gyro + meas_j.gyro) - bias_g;
    Quat dq = Utility::DeltaQ(mid_gyro * dt);
    state_j.q_wi = (state_i.q_wi * dq).normalized();

    // Propagate nominal velocity.
    mid_accel = 0.5f * (state_i.q_wi * (meas_i.accel - bias_a) + state_j.q_wi * (meas_j.accel - bias_a));
    state_j.v_wi = state_i.v_wi + (mid_accel - gravity) * dt;

    // Propagate nominal position.
    state_j.p_wi = state_i.p_wi + 0.5f * (state_i.v_wi + state_j.v_wi) * dt;

    return true;
}

bool Imu::PropagateNominalStateCovariance(const ImuMeasurement &meas_i,
                                          const ImuMeasurement &meas_j,
                                          const Mat &cov_i,
                                          Mat &cov_j) {

    return true;
}

bool Imu::PropagateResidualStateCovariance(const ImuMeasurement &meas_i,
                                           const ImuMeasurement &meas_j,
                                           const Mat &cov_i,
                                           Mat &cov_j) {

    return true;
}

bool Imu::PropagetePreintegrationBlock(const ImuMeasurement &meas_i,
                                       const ImuMeasurement &meas_j,
                                       ImuPreintegrateBlock &block) {

    return true;
}

}
