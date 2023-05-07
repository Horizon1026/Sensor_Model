#ifndef _SENSOR_MODEL_IMU_BASIC_H_
#define _SENSOR_MODEL_IMU_BASIC_H_

#include "datatype_basic.h"
#include "math_kinematics.h"
#include "imu_state.h"

namespace SENSOR_MODEL {

struct ImuMeasurement {
    Vec3 accel = Vec3::Zero();
    Vec3 gyro = Vec3::Zero();
    float time_stamp = 0.0f;
};

enum ImuStateIndex : uint8_t {
    kPosition = 0,
    kVelocity = 3,
    kRotation = 6,
    kBiasAccel = 9,
    kBiasGyro = 12,
    kGravity = 15,
};

class Imu {

public:
    Imu() = default;
    virtual ~Imu() = default;

    void PropagateNominalState(const ImuMeasurement &measurement);

    void PropagateNominalStateCovariance(const ImuMeasurement &measurement);

    void PropagateResidualStateCovariance(const ImuMeasurement &measurement);

    void PropagetePreintegrationBlock(const ImuMeasurement &measurement);

    // Reference for member variables.
    float &noise_accel() { return noise_accel_; }
    float &noise_gyro() { return noise_gyro_; }
    float &random_walk_accel() { return random_walk_accel_; }
    float &random_walk_gyro() { return random_walk_gyro_; }

private:
    float noise_accel_ = 0.0f;
    float noise_gyro_ = 0.0f;
    float random_walk_accel_ = 0.0f;
    float random_walk_gyro_ = 0.0f;

};

}

#endif
