#ifndef _SENSOR_MODEL_CAMERA_BASIC_H_
#define _SENSOR_MODEL_CAMERA_BASIC_H_

#include "datatype_basic.h"

namespace SensorModel {

enum class UndistortMethod : uint8_t {
    kGradientDesent = 0,
    kFixePointIteration = 1,
};

struct CameraModelOptions {
    UndistortMethod kUndistortMethod = UndistortMethod::kGradientDesent;
    uint32_t kMaxIterationForUndistortion = 10;
    float kMaxIterateStepLengthForUndistortion = 1e-6f;

};

class CameraBasic {

public:
    CameraBasic() = default;
    CameraBasic(float fx, float fy, float cx, float cy) : fx_(fx), fy_(fy), cx_(cx), cy_(cy) {}
    virtual ~CameraBasic() = default;
    CameraBasic(const CameraBasic &camera_basic) = delete;

public:
    // Lift 3d point in camera frame on normalized plane.
    virtual void LiftToNormalizedPlane(const Vec3 p_c, Vec2 &norm_xy) = 0;

    // Lift 2d point in normalized plane on image plane.
    inline void LiftToImagePlane(const Vec2 norm_xy, Vec2 &pixel_uv);

    // Lift 2d point in image plane back on normalized plane.
    inline void LiftBackToNormalizedPlane(const Vec2 pixel_uv, Vec2 &norm_xy);

	void SetMatrixK(float fx, float fy, float cx, float cy);

    const float &fx() const { return fx_; }
    const float &fy() const { return fy_; }
    const float &cx() const { return cx_; }
    const float &cy() const { return cy_; }

    CameraModelOptions &options() { return options_; }

private:
    float fx_ = 0.0f;
    float fy_ = 0.0f;
    float cx_ = 0.0f;
    float cy_ = 0.0f;

    CameraModelOptions options_;

};

}

#endif // end of _SENSOR_MODEL_CAMERA_BASIC_H_
