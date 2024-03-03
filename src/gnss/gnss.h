#ifndef _SENSOR_MODEL_GNSS_BASIC_H_
#define _SENSOR_MODEL_GNSS_BASIC_H_

#include "datatype_basic.h"
#include "math_kinematics.h"

#include "gnss_measurement.h"

namespace SENSOR_MODEL {

/* Class Gnss Model Declaration. */
class Gnss {

public:
    Gnss() = default;
    virtual ~Gnss() = default;

    Vec3 ConvertLlaToNed(const GnssMeasurement &origin_lla, const GnssMeasurement &lla);

private:

};

}

#endif // end of _SENSOR_MODEL_GNSS_BASIC_H_