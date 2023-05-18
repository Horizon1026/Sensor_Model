#ifndef _SENSOR_MODEL_VISUAL_OBSERVE_H_
#define _SENSOR_MODEL_VISUAL_OBSERVE_H_

#include "datatype_basic.h"
#include "unordered_map"

namespace SENSOR_MODEL {

using VisualPointFeatureObserve = Vec2;
using VisualPointFeatureObserveMultiView = std::unordered_map<int32_t, Vec2>;

using VisualLineFeatureObserve = Vec6;
using VisualLineFeatureObserveMultiView = std::unordered_map<int32_t, Vec6>;

}

#endif // end of _SENSOR_MODEL_VISUAL_OBSERVE_H_
