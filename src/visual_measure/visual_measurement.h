#ifndef _SENSOR_MODEL_VISUAL_MEASUREMENT_H_
#define _SENSOR_MODEL_VISUAL_MEASUREMENT_H_

#include "basic_type.h"

namespace sensor_model {

/* Visual measurements of points. */
struct PointsObservePerView {
    Vec2 raw_pixel_uv = Vec2::Zero();
    Vec2 rectified_norm_xy = Vec2::Zero();
    Vec3 bearing_xyz = Vec3::Zero();
};
using PointsObservePerFrame = std::vector<PointsObservePerView>;
struct VisualPointsMeasure {
    bool is_current_keyframe = true;
    double time_stamp_s = -1.0;
    int32_t direction_id = 0;
    // Basic visual measures.
    std::vector<uint32_t> features_id;
    std::vector<PointsObservePerFrame> observes_per_frame;  // observes_per_frame[feature_id][camera_id] -> PointsObservePerView.
    std::vector<Vec> feature_descriptors;
    // Measure for loop detection.
    Vec image_descriptor;
    MatImg image_mat;
    // Measure from ToF.
    float image_center_depth_m = 0;
    // Params for rolling shutter camera.
    double rs_read_out_time_s = 0.0;
    int32_t rs_mid_row_index = 0;
};

/* Visual measurements of lines. */
struct LinesObservePerView {
    Vec4 raw_pixel_uv = Vec4::Zero();
    Vec4 rectified_norm_xy = Vec4::Zero();
};
using LinesObservePerFrame = std::vector<LinesObservePerView>;
struct VisualLinesMeasure {
    bool is_current_keyframe = true;
    double time_stamp_s = -1.0;
    int32_t direction_id = 0;
    // Basic visual measures.
    std::vector<uint32_t> features_id;
    std::vector<LinesObservePerFrame> observes_per_frame;  // observes_per_frame[feature_id][camera_id] -> PointsObservePerView.
    std::vector<Vec> feature_descriptors;
    // Measure for loop detection.
    Vec image_descriptor;
    MatImg image_mat;
    // Measure from ToF.
    float image_center_depth_m = 0;
    // Params for rolling shutter camera.
    double rs_read_out_time_s = 0.0;
    int32_t rs_mid_row_index = 0;
};

/* Visual measurements of all types. */
struct VisualMixMeasure {
    bool is_current_keyframe = true;
    double time_stamp_s = -1.0;
    int32_t direction_id = 0;
    // Basic visual measures.
    std::vector<uint32_t> points_id;
    std::vector<PointsObservePerFrame> points_observes_per_frame;  // points_observes_per_frame[feature_id][camera_id] -> PointsObservePerView.
    std::vector<Vec> point_descriptors;
    std::vector<uint32_t> lines_id;
    std::vector<LinesObservePerFrame> lines_observes_per_frame;  // lines_observes_per_frame[feature_id][camera_id] -> LinesObservePerView.
    std::vector<Vec> line_descriptors;
    // Measure for loop detection.
    Vec image_descriptor;
    MatImg image_mat;
    // Measure from ToF.
    float image_center_depth_m = 0;
    // Params for rolling shutter camera.
    double rs_read_out_time_s = 0.0;
    int32_t rs_mid_row_index = 0;
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_VISUAL_MEASUREMENT_H_
