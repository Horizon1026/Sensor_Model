#ifndef _SENSOR_MODEL_CAMERA_BASIC_H_
#define _SENSOR_MODEL_CAMERA_BASIC_H_

#include "basic_type.h"
#include "datatype_image.h"

namespace sensor_model {

class CameraBasic {

public:
    enum class UndistortMethod : uint8_t {
        kGradientDesent = 0,
        kFixedPointIteration = 1,
        kBisection = 2,
        kNewtonIteration = 3,
    };

    struct Options {
        UndistortMethod kUndistortMethod = UndistortMethod::kGradientDesent;
        uint32_t kMaxIterationForUndistortion = 10;
        float kMaxIterateStepLengthForUndistortion = 1e-6f;
    };

public:
    CameraBasic() = default;
    CameraBasic(float fx, float fy, float cx, float cy): fx_(fx), fy_(fy), cx_(cx), cy_(cy) {}
    virtual ~CameraBasic() = default;
    CameraBasic(const CameraBasic &camera_basic) = delete;

public:
    // Lift 2d point in normalized plane to other coordinate systems.
    void LiftFromNormalizedPlaneToUnitSphere(const Vec2 norm_xy, Vec3 &sphere_xyz);
    void LiftFromNormalizedPlaneToImagePlane(const Vec2 norm_xy, Vec2 &pixel_uv);
    void LiftFromNormalizedPlaneToBearingVector(const Vec2 norm_xy, Vec3 &bearing_vector);

    // Lift 3d point in unit sphere to other coordinate systems.
    void LiftFromUnitSphereToNormalizedPlane(const Vec3 sphere_xyz, Vec2 &norm_xy);
    void LiftFromUnitSphereToBearingVector(const Vec3 sphere_xyz, Vec3 &bearing_vector);
    void LiftFromUnitSphereToImagePlane(const Vec3 sphere_xyz, Vec2 &pixel_uv);

    // Lift 3d point in bearing vector to other coordinate systems.
    void LiftFromBearingVectorToNormalizedPlane(const Vec3 bearing_vector, Vec2 &norm_xy);
    void LiftFromBearingVectorToUnitSphere(const Vec3 bearing_vector, Vec3 &sphere_xyz);
    void LiftFromBearingVectorToImagePlane(const Vec3 bearing_vector, Vec2 &pixel_uv);

    // Lift 2d point in image plane to other coordinate systems.
    void LiftFromImagePlaneToNormalizedPlane(const Vec2 pixel_uv, Vec2 &norm_xy);
    void LiftFromImagePlaneToBearingVector(const Vec2 pixel_uv, Vec3 &bearing_vector);
    void LiftFromImagePlaneToUnitSphere(const Vec2 pixel_uv, Vec3 &sphere_xyz);

    // Lift 2d point in raw image plane to undistorted normalized plane.
    bool LiftFromRawImagePlaneToUndistortedNormalizedPlane(const Vec2 pixel_uv, Vec2 &undistort_xy);

    // Lift 3d point in camera frame to normalized plane.
    void LiftFromCameraFrameToNormalizedPlane(const Vec3 p_c, Vec2 &norm_xy);

    // Do distortion and undistortion on normalized plane.
    virtual bool DistortOnNormalizedPlane(const Vec2 undistort_xy, Vec2 &distort_xy);
    virtual bool UndistortOnNormalizedPlane(const Vec2 distort_xy, Vec2 &undistort_xy);

    // Do distortion and undistortion on image plane.
    bool DistortOnImagePlane(const Vec2 undistort_uv, Vec2 &distort_uv);
    bool UndistortOnImagePlane(const Vec2 distort_uv, Vec2 &undistort_uv);

    // Undistort image.
    bool CorrectDistortedImage(const GrayImage &raw_image, GrayImage &corrected_image, float scale = 1.0f);

    // Set intrinsic and distortion parameters.
    void SetIntrinsicParameter(float fx, float fy, float cx, float cy);
    virtual void SetDistortionParameter(const std::vector<float> &params) {};

    // Reference for member variables.
    Options &options() { return options_; }

    // Const reference for member variables.
    const Options &options() const { return options_; }
    const float &fx() const { return fx_; }
    const float &fy() const { return fy_; }
    const float &cx() const { return cx_; }
    const float &cy() const { return cy_; }

private:
    float fx_ = 0.0f;
    float fy_ = 0.0f;
    float cx_ = 0.0f;
    float cy_ = 0.0f;

    Options options_;
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_CAMERA_BASIC_H_
