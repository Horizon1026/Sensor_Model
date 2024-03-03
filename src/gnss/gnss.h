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

    TVec3<double> ConvertLlaToNed(const GnssMeasurement &origin_lla, const GnssMeasurement &lla);
    GnssMeasurement ConvertNedToLla(const GnssMeasurement &origin_lla, const TVec3<double> &ned);

private:
    double FormatDegree(const double abnormal_degree);

};

}

#endif // end of _SENSOR_MODEL_GNSS_BASIC_H_