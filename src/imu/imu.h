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
    int32_t kImuStateSize = 15;
};

/* Class IMU model Declaration. */
class Imu {

public:
    Imu() = default;
    virtual ~Imu() = default;

    bool PropagateNominalState(const ImuMeasurement &measurement,
                               const ImuState &state_i,
                               ImuState &state_j);

    bool PropagateNominalStateCovariance(const ImuMeasurement &measurement,
                                         const Mat &covariance_i,
                                         Mat &covariance_j);

    bool PropagateResidualStateCovariance(const ImuMeasurement &measurement);

    bool PropagetePreintegrationBlock(const ImuMeasurement &measurement);

    // Reference for member variables.
    ImuModelOptions &options() { return options_; }

private:
    ImuModelOptions options_;

};

}

#endif
