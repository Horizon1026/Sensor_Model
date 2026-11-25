#ifndef _SENSOR_MODEL_TOF_MODEL_H_
#define _SENSOR_MODEL_TOF_MODEL_H_

#include "tof_measurement.h"

#include "string"
#include "vector"

namespace sensor_model {

/* Class Tof Model Declaration. */
class Tof {

public:
    Tof() = default;
    virtual ~Tof() = default;

    // CSV: timestamp[ns], is_depth_valid, depth[m].
    static bool ParseMeasuresFromFile(const std::string &file_path, const double &time_stamp_offset_s, std::vector<TofMeasurement> &measures);
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_TOF_MODEL_H_
