#ifndef _SENSOR_MODEL_VISUAL_FRAME_H_
#define _SENSOR_MODEL_VISUAL_FRAME_H_

#include "datatype_basic.h"
#include "unordered_map"

namespace SENSOR_MODEL {

/* Class Visual Frame Declaration. */
template <typename FeatureType>
class VisualFrame {

public:
    VisualFrame() = default;
    virtual ~VisualFrame() = default;

private:
    // Visual frame id.
    uint32_t id_ = 0;

    // Camera position, velocity and attitude combined with this frame.
    Quat q_wc_ = Quat::Identity();
    Vec3 p_wc_ = Vec3::Zero();
    Vec3 v_wc_ = Vec3::Zero();
    float time_stamp_s_ = 0.0f;

    // Features observed by this frame.
    std::unordered_map<uint32_t, FeatureType *> features_ = {};

};

}

#endif // end of _SENSOR_MODEL_VISUAL_FRAME_H_
