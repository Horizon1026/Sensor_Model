#ifndef _SENSOR_MODEL_BASIC_CAMERA_MODEL_H_
#define _SENSOR_MODEL_BASIC_CAMERA_MODEL_H_

#include "basic_type.h"
#include "datatype_image.h"

#include <string>
#include <vector>

namespace sensor_model {

// Shared camera state and projection interface. Sphere conventions and valid
// projection domains are defined by each concrete camera model.
class BasicCameraModel {
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
    virtual ~BasicCameraModel() = default;
    BasicCameraModel(const BasicCameraModel &) = delete;
    virtual std::string CameraModelName() const = 0;

    // Lift 2d point in normalized plane to other coordinate systems.
    virtual void LiftFromNormalizedPlaneToUnitSphere(const Vec2 norm_xy, Vec3 &sphere_xyz) const = 0;
    void LiftFromNormalizedPlaneToImagePlane(const Vec2 norm_xy, Vec2 &pixel_uv) const;
    virtual void LiftFromNormalizedPlaneToBearingVector(const Vec2 norm_xy, Vec3 &bearing_vector) const = 0;

    // Lift 3d point in unit sphere to other coordinate systems.
    virtual void LiftFromUnitSphereToNormalizedPlane(const Vec3 sphere_xyz, Vec2 &norm_xy) const = 0;
    void LiftFromUnitSphereToBearingVector(const Vec3 sphere_xyz, Vec3 &bearing_vector) const;
    void LiftFromUnitSphereToImagePlane(const Vec3 sphere_xyz, Vec2 &pixel_uv) const;

    // Lift 3d point in bearing vector to other coordinate systems.
    virtual void LiftFromBearingVectorToNormalizedPlane(const Vec3 bearing_vector, Vec2 &norm_xy) const = 0;
    void LiftFromBearingVectorToUnitSphere(const Vec3 bearing_vector, Vec3 &sphere_xyz) const;
    void LiftFromBearingVectorToImagePlane(const Vec3 bearing_vector, Vec2 &pixel_uv) const;

    // Lift 2d point in image plane to other coordinate systems.
    virtual void LiftFromImagePlaneToNormalizedPlane(const Vec2 pixel_uv, Vec2 &norm_xy) const = 0;
    void LiftFromImagePlaneToBearingVector(const Vec2 pixel_uv, Vec3 &bearing_vector) const;
    void LiftFromImagePlaneToUnitSphere(const Vec2 pixel_uv, Vec3 &sphere_xyz) const;

    // Lift 2d point in raw image plane to undistorted normalized plane.
    virtual bool LiftFromRawImagePlaneToUndistortedNormalizedPlane(const Vec2 pixel_uv, Vec2 &undistort_xy) const = 0;

    // Lift 3d point in camera frame to normalized plane.
    virtual void LiftFromCameraFrameToNormalizedPlane(const Vec3 p_c, Vec2 &norm_xy) const = 0;

    // Do distortion and undistortion on normalized plane.
    virtual bool DistortOnNormalizedPlane(const Vec2 undistort_xy, Vec2 &distort_xy) const = 0;
    virtual bool UndistortOnNormalizedPlane(const Vec2 distort_xy, Vec2 &undistort_xy) const = 0;

    // Do distortion and undistortion on image plane.
    virtual bool DistortOnImagePlane(const Vec2 undistort_uv, Vec2 &distort_uv) const = 0;
    virtual bool UndistortOnImagePlane(const Vec2 distort_uv, Vec2 &undistort_uv) const = 0;

    // Undistort image.
    virtual bool CorrectDistortedImage(const GrayImage &raw_image, GrayImage &corrected_image, float scale = 1.0f) const = 0;

    // Set intrinsic and distortion parameters.
    void SetImageSize(int32_t image_rows, int32_t image_cols);
    virtual void SetDistortionParameter(const std::vector<float> &params) {};
    virtual void GetDistortionParameter(std::vector<float> &params) const { params.clear(); }

    // Reference for member variables.
    Options &options() { return options_; }

    // Const reference for member variables.
    const Options &options() const { return options_; }
    const float &fx() const { return fx_; }
    const float &fy() const { return fy_; }
    const float &cx() const { return cx_; }
    const float &cy() const { return cy_; }
    const int32_t &image_rows() const { return image_rows_; }
    const int32_t &image_cols() const { return image_cols_; }

protected:
    BasicCameraModel() = default;
    BasicCameraModel(float fx, float fy, float cx, float cy, int32_t image_rows = 0, int32_t image_cols = 0);
    // Concrete models expose their own intrinsic setters (omni also requires xi).
    void SetCommonIntrinsicParameter(float fx, float fy, float cx, float cy);

private:
    Options options_;

    float fx_ = 0.0f;
    float fy_ = 0.0f;
    float cx_ = 0.0f;
    float cy_ = 0.0f;

    int32_t image_rows_ = 0;
    int32_t image_cols_ = 0;
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_BASIC_CAMERA_MODEL_H_
