#ifndef _SENSOR_MODEL_CAMERA_OMNI_H_
#define _SENSOR_MODEL_CAMERA_OMNI_H_

#include "basic_camera_model.h"

namespace sensor_model {

// Unified omnidirectional camera model (Mei):
// normalized_xy = p_c.xy / (p_c.z + xi * ||p_c||).
// Unit-sphere coordinates and bearing vectors both represent physical rays.
// For xi > 1 only the injective branch z / ||p_c|| > -1 / xi is used.
// Invalid lifts return NaNs; bool operations return false on invalid input.
class CameraOmni: public BasicCameraModel {
public:
    CameraOmni() = default;
    CameraOmni(float fx, float fy, float cx, float cy, float xi = 0.0f);
    CameraOmni(float fx, float fy, float cx, float cy, float xi, int32_t image_rows, int32_t image_cols);
    virtual ~CameraOmni() = default;
    CameraOmni(const CameraOmni &) = delete;

public:
    virtual std::string CameraModelName() const override { return "Omni-Rectify"; }

    // Projection and lens operations use the common interface in BasicCameraModel.
    virtual void LiftFromNormalizedPlaneToUnitSphere(const Vec2 norm_xy, Vec3 &sphere_xyz) const override;
    virtual void LiftFromNormalizedPlaneToBearingVector(const Vec2 norm_xy, Vec3 &bearing_vector) const override;
    virtual void LiftFromUnitSphereToNormalizedPlane(const Vec3 sphere_xyz, Vec2 &norm_xy) const override;
    virtual void LiftFromBearingVectorToNormalizedPlane(const Vec3 bearing_vector, Vec2 &norm_xy) const override;
    virtual void LiftFromImagePlaneToNormalizedPlane(const Vec2 pixel_uv, Vec2 &norm_xy) const override;
    virtual bool LiftFromRawImagePlaneToUndistortedNormalizedPlane(const Vec2 pixel_uv, Vec2 &undistort_xy) const override;
    virtual void LiftFromCameraFrameToNormalizedPlane(const Vec3 p_c, Vec2 &norm_xy) const override;
    virtual bool DistortOnNormalizedPlane(const Vec2 undistort_xy, Vec2 &distort_xy) const override;
    virtual bool UndistortOnNormalizedPlane(const Vec2 distort_xy, Vec2 &undistort_xy) const override;
    virtual bool DistortOnImagePlane(const Vec2 undistort_uv, Vec2 &distort_uv) const override;
    virtual bool UndistortOnImagePlane(const Vec2 distort_uv, Vec2 &undistort_uv) const override;
    // Removes lens distortion while retaining the omni projection and xi.
    // Input/output buffers must be distinct and have the same dimensions.
    virtual bool CorrectDistortedImage(const GrayImage &raw_image, GrayImage &corrected_image, float scale = 1.0f) const override;
    // xi must be finite and nonnegative, otherwise throws std::invalid_argument.
    void SetIntrinsicParameter(float fx, float fy, float cx, float cy, float xi);

    // Const reference for member variables.
    const float &xi() const { return xi_; }

private:
    float xi_ = 0.0f;
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_CAMERA_OMNI_H_
