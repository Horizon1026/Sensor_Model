#ifndef _SENSOR_MODEL_VISUAL_MEASURE_H_
#define _SENSOR_MODEL_VISUAL_MEASURE_H_

#include "visual_measurement.h"

#include "string"

namespace sensor_model {

class VisualMeasure {

public:
    // Frame CSV: hash_id, feature_id, world point, then pixel, normalized coordinate, bearing and depth per view.
    // Timestamp, direction and rolling-shutter parameters come from the frame index and are preserved.
    static bool ParseOneMeasureFromFile(const std::string &file_path, VisualMixMeasure &measure);
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_VISUAL_MEASURE_H_
