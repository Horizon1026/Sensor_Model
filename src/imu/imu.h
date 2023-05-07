#ifndef _SENSOR_MODEL_IMU_BASIC_H_
#define _SENSOR_MODEL_IMU_BASIC_H_

#include "datatype_basic.h"
#include "math_kinematics.h"

namespace SENSOR_MODEL {

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

};

}

#endif
