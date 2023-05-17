#ifndef _SENSOR_MODEL_VISUAL_FEATURE_H_
#define _SENSOR_MODEL_VISUAL_FEATURE_H_

#include "datatype_basic.h"

namespace SENSOR_MODEL {

enum FeatureSolvedStatus : uint8_t {
    kUnsolved = 0,
    kSolved = 1,
    kOutliers = 2,
};

/* Class Visual Feature Decalaration. */
template <typename ParamType, typename ObserveType>
class VisualFeature {

public:
    VisualFeature() = default;
    virtual ~VisualFeature() = default;

private:
    // Visual feature id.
    uint32_t id_ = 0;

    // Id of frame that firstly observe this feature.
    uint32_t first_frame_id_ = 0;

    // Observations in each visual frame. For example, feature point's observe in a mono rectify frame is Eigen::Vector2f.
    std::vector<ObserveType> observes_ = {};

    // Feature parameter can be solved. For example, feature point's parameter type is Eigen::Vector3f.
    ParamType param_;

    // Solved type of this feature.
    FeatureSolvedStatus status_ = FeatureSolvedStatus::kUnsolved;

};

}

#endif
