#ifndef _SENSOR_MODEL_CAMERA_OMNI_H_
#define _SENSOR_MODEL_CAMERA_OMNI_H_

#include "basic_type.h"
#include "datatype_image.h"

#include "string"
#include "vector"

namespace sensor_model {

// Unified omnidirectional camera model (Mei):
// normalized_xy = p_c.xy / (p_c.z + xi * ||p_c||).
// Unit-sphere coordinates and bearing vectors both represent physical rays.
// For xi > 1 only the injective branch z / ||p_c|| > -1 / xi is used.
// Invalid lifts return NaNs; bool operations return false on invalid input.
class CameraOmni {

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
    CameraOmni() = default;
    CameraOmni(float fx, float fy, float cx, float cy, float xi = 0.0f);
    CameraOmni(float fx, float fy, float cx, float cy, float xi, int32_t image_rows, int32_t image_cols);
    virtual ~CameraOmni() = default;
    CameraOmni(const CameraOmni &camera_basic) = delete;

public:
    virtual std::string CameraModelName() const { return "Omni-Rectify"; }

    // Lift 2d point in normalized plane to other coordinate systems.
    void LiftFromNormalizedPlaneToUnitSphere(const Vec2 norm_xy, Vec3 &sphere_xyz) const;
    void LiftFromNormalizedPlaneToImagePlane(const Vec2 norm_xy, Vec2 &pixel_uv) const;
    void LiftFromNormalizedPlaneToBearingVector(const Vec2 norm_xy, Vec3 &bearing_vector) const;

    // Lift 3d point in unit sphere to other coordinate systems.
    void LiftFromUnitSphereToNormalizedPlane(const Vec3 sphere_xyz, Vec2 &norm_xy) const;
    void LiftFromUnitSphereToBearingVector(const Vec3 sphere_xyz, Vec3 &bearing_vector) const;
    void LiftFromUnitSphereToImagePlane(const Vec3 sphere_xyz, Vec2 &pixel_uv) const;

    // Lift 3d point in bearing vector to other coordinate systems.
    void LiftFromBearingVectorToNormalizedPlane(const Vec3 bearing_vector, Vec2 &norm_xy) const;
    void LiftFromBearingVectorToUnitSphere(const Vec3 bearing_vector, Vec3 &sphere_xyz) const;
    void LiftFromBearingVectorToImagePlane(const Vec3 bearing_vector, Vec2 &pixel_uv) const;

    // Lift 2d point in image plane to other coordinate systems.
    void LiftFromImagePlaneToNormalizedPlane(const Vec2 pixel_uv, Vec2 &norm_xy) const;
    void LiftFromImagePlaneToBearingVector(const Vec2 pixel_uv, Vec3 &bearing_vector) const;
    void LiftFromImagePlaneToUnitSphere(const Vec2 pixel_uv, Vec3 &sphere_xyz) const;

    // Lift 2d point in raw image plane to undistorted normalized plane.
    bool LiftFromRawImagePlaneToUndistortedNormalizedPlane(const Vec2 pixel_uv, Vec2 &undistort_xy) const;

    // Lift 3d point in camera frame to normalized plane.
    void LiftFromCameraFrameToNormalizedPlane(const Vec3 p_c, Vec2 &norm_xy) const;

    // Do distortion and undistortion on normalized plane.
    virtual bool DistortOnNormalizedPlane(const Vec2 undistort_xy, Vec2 &distort_xy) const;
    virtual bool UndistortOnNormalizedPlane(const Vec2 distort_xy, Vec2 &undistort_xy) const;

    // Do distortion and undistortion on image plane.
    bool DistortOnImagePlane(const Vec2 undistort_uv, Vec2 &distort_uv) const;
    bool UndistortOnImagePlane(const Vec2 distort_uv, Vec2 &undistort_uv) const;

    // Remove lens distortion while retaining the omni projection and xi.
    // Input/output buffers must be distinct and have the same dimensions.
    bool CorrectDistortedImage(const GrayImage &raw_image, GrayImage &corrected_image, float scale = 1.0f) const;

    // Set intrinsic and distortion parameters.
    void SetImageSize(int32_t image_rows, int32_t image_cols);
    // xi must be finite and nonnegative, otherwise throws std::invalid_argument.
    void SetIntrinsicParameter(float fx, float fy, float cx, float cy, float xi);
    virtual void SetDistortionParameter(const std::vector<float> &params) {};
    virtual void GetDistortionParameter(std::vector<float> &params) const { params.clear(); }

    // Reference for member variables.
    Options &options() { return options_; }

    // Const reference for member variables.
    const Options &options() const { return options_; }
    const float &xi() const { return xi_; }
    const float &fx() const { return fx_; }
    const float &fy() const { return fy_; }
    const float &cx() const { return cx_; }
    const float &cy() const { return cy_; }
    const int32_t &image_rows() const { return image_rows_; }
    const int32_t &image_cols() const { return image_cols_; }

private:
    Options options_;

    float xi_ = 0.0f;
    float fx_ = 0.0f;
    float fy_ = 0.0f;
    float cx_ = 0.0f;
    float cy_ = 0.0f;

    int32_t image_rows_ = 0;
    int32_t image_cols_ = 0;
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_CAMERA_OMNI_H_
