#ifndef _SENSOR_MODEL_TOF_MODEL_H_
#define _SENSOR_MODEL_TOF_MODEL_H_

#include "tof_measurement.h"

namespace sensor_model {

/* Class Tof Model Declaration. */
class Tof {

public:
    Tof() = default;
    virtual ~Tof() = default;
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_TOF_MODEL_H_
