#ifndef _SENSOR_MODEL_LIDAR_MEASUREMENT_H_
#define _SENSOR_MODEL_LIDAR_MEASUREMENT_H_

#include "basic_type.h"

namespace SENSOR_MODEL {

/* Measurement of Lidar. */
struct LidarMeasurement {
    float time_stamp_s = 0.0f;
    std::vector<float> time_stamp_s_of_points;
    std::vector<Vec3> raw_points;
    std::vector<Vec3> undistorted_points;
};

}

#endif // end of _SENSOR_MODEL_LIDAR_MEASUREMENT_H_
