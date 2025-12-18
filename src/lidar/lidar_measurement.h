#ifndef _SENSOR_MODEL_LIDAR_MEASUREMENT_H_
#define _SENSOR_MODEL_LIDAR_MEASUREMENT_H_

#include "basic_type.h"

namespace sensor_model {

/* Measurement of Lidar. */
struct LidarMeasurement {
    float time_stamp_s = 0.0f;
    std::vector<float> time_stamp_s_of_points;
    std::vector<Vec3> raw_points;
    std::vector<float> intensity_of_points;
    std::vector<Vec3> undistorted_points;
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_LIDAR_MEASUREMENT_H_
