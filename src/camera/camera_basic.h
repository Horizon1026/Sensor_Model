#ifndef _SENSOR_MODEL_CAMERA_BASIC_H_
#define _SENSOR_MODEL_CAMERA_BASIC_H_

#include "datatype_basic.h"
#include "datatype_image.h"

namespace SENSOR_MODEL {

enum class UndistortMethod : uint8_t {
    kGradientDesent = 0,
    kFixePointIteration = 1,
    kBisection = 2,
    kNewtonIteration = 3,
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
    // Lift 2d point in normalized plane on unit sphere.
    void LiftFromNormalizedPlaneToUnitSphere(const Vec2 norm_xy, Vec3 &sphere_xyz);
    // Lift 3d point in unit sphere on normalized plane.
    void LiftFromNormalizedPlaneToUnitSphere(const Vec3 sphere_xyz, Vec2 &norm_xy);

    // Lift 2d point in normalized plane on image plane.
    virtual void LiftFromNormalizedPlaneToImagePlane(const Vec2 norm_xy, Vec2 &pixel_uv);
    // Lift 2d point in image plane back on normalized plane.
    virtual void LiftFromImagePlaneToNormalizedPlane(const Vec2 pixel_uv, Vec2 &norm_xy);

    // Lift 3d point in camera frame on normalized plane.
    void LiftFromCameraFrameToNormalizedPlane(const Vec3 p_c, Vec2 &norm_xy);
    // Lift 3d point in camera frame on normalized plane, and do undistortion.
    bool LiftFromCameraFrameToNormalizedPlaneAndUndistort(const Vec2 pixel_uv, Vec2 &undistort_xy);

    // Do distortion and undistortion on normalized plane.
    virtual bool DistortOnNormalizedPlane(const Vec2 undistort_xy, Vec2 &distort_xy) { return false; }
    virtual bool UndistortOnNormalizedPlane(const Vec2 distort_xy, Vec2 &undistort_xy) { return false; }

    // Do distortion and undistortion on image plane.
    bool DistortOnImagePlane(const Vec2 undistort_uv, Vec2 &distort_uv);
    bool UndistortOnImagePlane(const Vec2 distort_uv, Vec2 &undistort_uv);

    // Undistort image.
    bool CorrectDistortedImage(const GrayImage &raw_image, GrayImage &corrected_image, float scale = 1.0f);

    // Set intrinsic and distortion parameters.
    void SetIntrinsicParameter(float fx, float fy, float cx, float cy);
    virtual void SetDistortionParameter(const std::vector<float> &params) {};

    // Reference for member variables.
    CameraModelOptions &options() { return options_; }

    // Const reference for member variables.
    const CameraModelOptions &options() const { return options_; }
    const float &fx() const { return fx_; }
    const float &fy() const { return fy_; }
    const float &cx() const { return cx_; }
    const float &cy() const { return cy_; }

private:
    float fx_ = 0.0f;
    float fy_ = 0.0f;
    float cx_ = 0.0f;
    float cy_ = 0.0f;

    CameraModelOptions options_;

};

}

#endif // end of _SENSOR_MODEL_CAMERA_BASIC_H_
