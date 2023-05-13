#ifndef _SENSOR_MODEL_IMU_BASIC_H_
#define _SENSOR_MODEL_IMU_BASIC_H_

#include "datatype_basic.h"
#include "math_kinematics.h"
#include "imu_state.h"
#include "imu_preintegrate.h"

namespace SENSOR_MODEL {

struct ImuModelOptions {
    float kAccelNoise = 1e-6f;
    float kGyroNoise = 1e-6f;
    float kAccelRandomWalk = 1e-6f;
    float kGyroRandomWalk = 1e-6f;
};

/* Class IMU model Declaration. */
class Imu {

public:
    Imu() = default;
    virtual ~Imu() = default;

    bool PropagateNominalState(const ImuMeasurement &meas_i,
                               const ImuMeasurement &meas_j,
                               const ImuState &state_i,
                               ImuState &state_j);

    bool PropagateNominalState(const ImuMeasurement &meas_i,
                               const ImuMeasurement &meas_j,
                               const ImuState &state_i,
                               ImuState &state_j,
                               Vec3 &mid_accel,
                               Vec3 &mid_gyro);

    bool PropagateNominalStateCovariance(const ImuMeasurement &meas_i,
                                         const ImuMeasurement &meas_j,
                                         const Vec3 &mid_accel,
                                         const Vec3 &mid_gyro,
                                         const ImuState &state_i,
                                         const ImuState &state_j,
                                         const Mat15 &cov_i,
                                         Mat15 &cov_j);

    bool PropagateResidualStateCovariance(const ImuMeasurement &meas_i,
                                          const ImuMeasurement &meas_j,
                                          const Vec3 &mid_accel,
                                          const Vec3 &mid_gyro,
                                          const ImuState &state_i,
                                          const ImuState &state_j,
                                          const Mat15 &cov_i,
                                          Mat15 &cov_j);

    // Reference for member variables.
    ImuModelOptions &options() { return options_; }

private:
    ImuModelOptions options_;

    // Sequence is na, ng, nwa, nwg
    Vec12 noise_sigma_ = Vec12::Ones() * 1e-6f;

};

}

#endif
