#ifndef _SENSOR_MODEL_CAMERA_PINHOLE_H_
#define _SENSOR_MODEL_CAMERA_PINHOLE_H_

#include "basic_camera_model.h"

namespace sensor_model {

class CameraPinhole: public BasicCameraModel {
public:
    CameraPinhole() = default;
    CameraPinhole(float fx, float fy, float cx, float cy): BasicCameraModel(fx, fy, cx, cy) {}
    CameraPinhole(float fx, float fy, float cx, float cy, int32_t image_rows, int32_t image_cols):
        BasicCameraModel(fx, fy, cx, cy, image_rows, image_cols) {}
    virtual ~CameraPinhole() = default;
    CameraPinhole(const CameraPinhole &) = delete;

public:
    virtual std::string CameraModelName() const override { return "Pinhole-Rectify"; }

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
    virtual bool CorrectDistortedImage(const GrayImage &raw_image, GrayImage &corrected_image, float scale = 1.0f) const override;
    void SetIntrinsicParameter(float fx, float fy, float cx, float cy);
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_CAMERA_PINHOLE_H_
