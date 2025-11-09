#ifndef _SENSOR_MODEL_CAMERA_MEASUREMENT_H_
#define _SENSOR_MODEL_CAMERA_MEASUREMENT_H_

#include "basic_type.h"
#include "datatype_image.h"

namespace sensor_model {

/* Measurement of camera. */
struct CameraMeasurement {
    MatImg image;
    float time_stamp_s = 0.0f;
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_CAMERA_MEASUREMENT_H_
