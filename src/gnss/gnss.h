#ifndef _SENSOR_MODEL_GNSS_MODEL_H_
#define _SENSOR_MODEL_GNSS_MODEL_H_

#include "gnss_measurement.h"

#include "basic_type.h"
#include "slam_basic_math.h"

#include "string"
#include "vector"

namespace sensor_model {

/* Class Gnss Model Declaration. */
class Gnss {

public:
    /* Options of Gnss Model. */
    struct Options {
        float kPositionNoiseSigma = 1.0f;
        float kVelocityNoiseSigma = 0.5f;
        float kAttitudeNoiseSigma = 3.0f;
    };

public:
    Gnss() = default;
    virtual ~Gnss() = default;

    // CSV: timestamp[ns], fixed_type, velocity_e, velocity_n, velocity_u, longitude[deg], latitude[deg], altitude[m].
    static bool ParseMeasuresFromFile(const std::string &file_path, const double &time_stamp_offset_s, std::vector<GnssMeasurement> &measures);

    // Process measurements to be state observations.
    TVec3<double> ConvertLlaToEnu(const GnssMeasurement &origin_lla, const GnssMeasurement &lla);
    GnssMeasurement ConvertEnuToLla(const GnssMeasurement &origin_lla, const TVec3<double> &enu);
    double FormatDegree(const double abnormal_degree);

    // Reference for member variables.
    Options &options() { return options_; }

    // Const reference for member variables.
    const Options &options() const { return options_; }

private:
    Options options_;
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_GNSS_MODEL_H_