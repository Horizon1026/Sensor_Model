#ifndef _SENSOR_MODEL_TOF_MEASUREMENT_H_
#define _SENSOR_MODEL_TOF_MEASUREMENT_H_

namespace sensor_model {

/* Measurement of single-point ToF. */
struct TofMeasurement {
    double time_stamp_s = 0.0;
    bool is_depth_valid = false;
    float depth_m = 0.0f;
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_TOF_MEASUREMENT_H_
