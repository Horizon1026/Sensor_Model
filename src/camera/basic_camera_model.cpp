#include "basic_camera_model.h"

namespace sensor_model {

BasicCameraModel::BasicCameraModel(float fx, float fy, float cx, float cy, int32_t image_rows, int32_t image_cols):
    fx_(fx), fy_(fy), cx_(cx), cy_(cy),
    image_rows_(image_rows > 0 ? image_rows : static_cast<int32_t>(cy * 2.0f)),
    image_cols_(image_cols > 0 ? image_cols : static_cast<int32_t>(cx * 2.0f)) {}

void BasicCameraModel::LiftFromNormalizedPlaneToImagePlane(const Vec2 norm_xy, Vec2 &pixel_uv) const {
    pixel_uv(0) = fx_ * norm_xy(0) + cx_;
    pixel_uv(1) = fy_ * norm_xy(1) + cy_;
}

void BasicCameraModel::LiftFromUnitSphereToBearingVector(const Vec3 sphere_xyz, Vec3 &bearing_vector) const {
    Vec2 norm_xy = Vec2::Zero();
    LiftFromUnitSphereToNormalizedPlane(sphere_xyz, norm_xy);
    LiftFromNormalizedPlaneToBearingVector(norm_xy, bearing_vector);
}

void BasicCameraModel::LiftFromUnitSphereToImagePlane(const Vec3 sphere_xyz, Vec2 &pixel_uv) const {
    Vec2 norm_xy = Vec2::Zero();
    LiftFromUnitSphereToNormalizedPlane(sphere_xyz, norm_xy);
    LiftFromNormalizedPlaneToImagePlane(norm_xy, pixel_uv);
}

void BasicCameraModel::LiftFromBearingVectorToUnitSphere(const Vec3 bearing_vector, Vec3 &sphere_xyz) const {
    Vec2 norm_xy = Vec2::Zero();
    LiftFromBearingVectorToNormalizedPlane(bearing_vector, norm_xy);
    LiftFromNormalizedPlaneToUnitSphere(norm_xy, sphere_xyz);
}

void BasicCameraModel::LiftFromBearingVectorToImagePlane(const Vec3 bearing_vector, Vec2 &pixel_uv) const {
    Vec2 norm_xy = Vec2::Zero();
    LiftFromBearingVectorToNormalizedPlane(bearing_vector, norm_xy);
    LiftFromNormalizedPlaneToImagePlane(norm_xy, pixel_uv);
}

void BasicCameraModel::LiftFromImagePlaneToBearingVector(const Vec2 pixel_uv, Vec3 &bearing_vector) const {
    Vec2 norm_xy = Vec2::Zero();
    LiftFromImagePlaneToNormalizedPlane(pixel_uv, norm_xy);
    LiftFromNormalizedPlaneToBearingVector(norm_xy, bearing_vector);
}

void BasicCameraModel::LiftFromImagePlaneToUnitSphere(const Vec2 pixel_uv, Vec3 &sphere_xyz) const {
    Vec2 norm_xy = Vec2::Zero();
    LiftFromImagePlaneToNormalizedPlane(pixel_uv, norm_xy);
    LiftFromNormalizedPlaneToUnitSphere(norm_xy, sphere_xyz);
}

void BasicCameraModel::SetImageSize(int32_t image_rows, int32_t image_cols) {
    image_rows_ = image_rows;
    image_cols_ = image_cols;
}

void BasicCameraModel::SetCommonIntrinsicParameter(float fx, float fy, float cx, float cy) {
    fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;
}

}  // namespace sensor_model
