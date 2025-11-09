#ifndef _SENSOR_MODEL_IMU_BASIC_H_
#define _SENSOR_MODEL_IMU_BASIC_H_

#include "basic_type.h"
#include "imu_measurement.h"
#include "imu_preintegrate.h"
#include "imu_state.h"
#include "slam_basic_math.h"

namespace SENSOR_MODEL {

/* Class IMU model Declaration. */
class Imu {

public:
    /* Options of IMU Model. */
    struct Options {
        float kAccelNoiseSigma = 1e-6f;
        float kGyroNoiseSigma = 1e-6f;
        float kAccelRandomWalkSigma = 1e-6f;
        float kGyroRandomWalkSigma = 1e-6f;
    };

public:
    Imu() = default;
    virtual ~Imu() = default;

    bool PropagateNominalState(const ImuMeasurement &meas_i, const ImuMeasurement &meas_j, const ImuState &state_i, ImuState &state_j);

    bool PropagateNominalState(const ImuMeasurement &meas_i, const ImuMeasurement &meas_j, const ImuState &state_i, ImuState &state_j, Vec3 &mid_accel,
                               Vec3 &mid_gyro);

    bool PropagateNominalStateCovariance(const ImuMeasurement &meas_i, const ImuMeasurement &meas_j, const Vec3 &mid_accel, const Vec3 &mid_gyro,
                                         const ImuState &state_i, const Mat15 &cov_i, Mat15 &cov_j);

    bool PropagateResidualStateCovariance(const ImuMeasurement &meas_i, const ImuMeasurement &meas_j, const Vec3 &mid_accel, const Vec3 &mid_gyro,
                                          const ImuState &state_i, const Mat15 &cov_i, Mat15 &cov_j);

    // Reference for member variables.
    Options &options() { return options_; }
    Vec12 &noise_sigma() { return noise_sigma_; }

    // Const reference for member variables.
    const Options &options() const { return options_; }
    const Vec12 &noise_sigma() const { return noise_sigma_; }

private:
    Options options_;

    // Sequence is na, ng, nwa, nwg
    Vec12 noise_sigma_ = Vec12::Ones() * 1e-6f;
};

}  // namespace SENSOR_MODEL

#endif
