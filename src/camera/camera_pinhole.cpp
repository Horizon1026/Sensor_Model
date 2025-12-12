#include "camera_pinhole.h"
#include "slam_basic_math.h"
#include "slam_operations.h"

namespace sensor_model {

// Lift 2d point in normalized plane to other coordinate systems.
void CameraPinhole::LiftFromNormalizedPlaneToUnitSphere(const Vec2 norm_xy, Vec3 &sphere_xyz) {
    const float yita = 2.0f / (1.0f + norm_xy.squaredNorm());
    sphere_xyz.head<2>() = norm_xy * yita;
    sphere_xyz.z() = yita - 1.0f;
}
void CameraPinhole::LiftFromNormalizedPlaneToImagePlane(const Vec2 norm_xy, Vec2 &pixel_uv) {
    pixel_uv(0) = fx_ * norm_xy(0) + cx_;
    pixel_uv(1) = fy_ * norm_xy(1) + cy_;
}
void CameraPinhole::LiftFromNormalizedPlaneToBearingVector(const Vec2 norm_xy, Vec3 &bearing_vector) {
    bearing_vector.head<2>() = norm_xy;
    bearing_vector.z() = 1.0f;
    bearing_vector.normalize();
}

// Lift 3d point in unit sphere to other coordinate systems.
void CameraPinhole::LiftFromUnitSphereToNormalizedPlane(const Vec3 sphere_xyz, Vec2 &norm_xy) {
    const float yita = sphere_xyz.z() + 1.0f;
    norm_xy = sphere_xyz.head<2>() / yita;
}
void CameraPinhole::LiftFromUnitSphereToBearingVector(const Vec3 sphere_xyz, Vec3 &bearing_vector) {
    Vec2 norm_xy = Vec2::Zero();
    LiftFromUnitSphereToNormalizedPlane(sphere_xyz, norm_xy);
    LiftFromNormalizedPlaneToBearingVector(norm_xy, bearing_vector);
}
void CameraPinhole::LiftFromUnitSphereToImagePlane(const Vec3 sphere_xyz, Vec2 &pixel_uv) {
    Vec2 norm_xy = Vec2::Zero();
    LiftFromUnitSphereToNormalizedPlane(sphere_xyz, norm_xy);
    LiftFromNormalizedPlaneToImagePlane(norm_xy, pixel_uv);
}

// Lift 3d point in bearing vector to other coordinate systems.
void CameraPinhole::LiftFromBearingVectorToNormalizedPlane(const Vec3 bearing_vector, Vec2 &norm_xy) {
    norm_xy = bearing_vector.head<2>() / bearing_vector.z();
}
void CameraPinhole::LiftFromBearingVectorToUnitSphere(const Vec3 bearing_vector, Vec3 &sphere_xyz) {
    Vec2 norm_xy = Vec2::Zero();
    LiftFromBearingVectorToNormalizedPlane(bearing_vector, norm_xy);
    LiftFromNormalizedPlaneToUnitSphere(norm_xy, sphere_xyz);
}
void CameraPinhole::LiftFromBearingVectorToImagePlane(const Vec3 bearing_vector, Vec2 &pixel_uv) {
    Vec2 norm_xy = Vec2::Zero();
    LiftFromBearingVectorToNormalizedPlane(bearing_vector, norm_xy);
    LiftFromNormalizedPlaneToImagePlane(norm_xy, pixel_uv);
}

// Lift 2d point in image plane back on normalized plane.
void CameraPinhole::LiftFromImagePlaneToNormalizedPlane(const Vec2 pixel_uv, Vec2 &norm_xy) {
    norm_xy(0) = (pixel_uv(0) - cx_) / fx_;
    norm_xy(1) = (pixel_uv(1) - cy_) / fy_;
}
void CameraPinhole::LiftFromImagePlaneToBearingVector(const Vec2 pixel_uv, Vec3 &bearing_vector) {
    Vec2 norm_xy = Vec2::Zero();
    LiftFromImagePlaneToNormalizedPlane(pixel_uv, norm_xy);
    LiftFromNormalizedPlaneToBearingVector(norm_xy, bearing_vector);
}
void CameraPinhole::LiftFromImagePlaneToUnitSphere(const Vec2 pixel_uv, Vec3 &sphere_xyz) {
    Vec2 norm_xy = Vec2::Zero();
    LiftFromImagePlaneToNormalizedPlane(pixel_uv, norm_xy);
    LiftFromNormalizedPlaneToUnitSphere(norm_xy, sphere_xyz);
}

// Lift 3d point in camera frame on normalized plane.
void CameraPinhole::LiftFromCameraFrameToNormalizedPlane(const Vec3 p_c, Vec2 &norm_xy) {
    if (p_c.z() < kZeroFloat) {
        norm_xy.setZero();
    } else {
        norm_xy.x() = p_c.x() / p_c.z();
        norm_xy.y() = p_c.y() / p_c.z();
    }
}

// Distort on normalized plane.
bool CameraPinhole::DistortOnNormalizedPlane(const Vec2 undistort_xy, Vec2 &distort_xy) {
    // For camera basic, view it as rectify image, so no distortion.
    distort_xy = undistort_xy;
    return true;
}

// Undistort on normalized plane.
bool CameraPinhole::UndistortOnNormalizedPlane(const Vec2 distort_xy, Vec2 &undistort_xy) {
    // For camera basic, view it as rectify image, so no undistortion.
    undistort_xy = distort_xy;
    return true;
}

// Lift 3d point in image plane on normalized plane, and do undistortion.
bool CameraPinhole::LiftFromRawImagePlaneToUndistortedNormalizedPlane(const Vec2 pixel_uv, Vec2 &undistort_xy) {
    Vec2 distort_xy = Vec2::Zero();
    LiftFromImagePlaneToNormalizedPlane(pixel_uv, distort_xy);
    return UndistortOnNormalizedPlane(distort_xy, undistort_xy);
}

bool CameraPinhole::DistortOnImagePlane(const Vec2 undistort_uv, Vec2 &distort_uv) {
    const Vec2 undistort_xy = Vec2((undistort_uv(0) - cx()) / fx(), (undistort_uv(1) - cy()) / fy());
    Vec2 distort_xy = Vec2::Zero();
    RETURN_FALSE_IF_FALSE(DistortOnNormalizedPlane(undistort_xy, distort_xy));
    LiftFromNormalizedPlaneToImagePlane(distort_xy, distort_uv);
    return true;
}

bool CameraPinhole::UndistortOnImagePlane(const Vec2 distort_uv, Vec2 &undistort_uv) {
    const Vec2 distort_xy = Vec2((distort_uv(0) - cx()) / fx(), (distort_uv(1) - cy()) / fy());
    Vec2 undistort_xy = Vec2::Zero();
    RETURN_FALSE_IF_FALSE(UndistortOnNormalizedPlane(distort_xy, undistort_xy));
    LiftFromNormalizedPlaneToImagePlane(undistort_xy, undistort_uv);
    return true;
}

// Undistort image.
bool CameraPinhole::CorrectDistortedImage(const GrayImage &raw_image, GrayImage &corrected_image, float scale) {
    RETURN_FALSE_IF(raw_image.data() == nullptr || corrected_image.data() == nullptr);
    RETURN_FALSE_IF(raw_image.cols() != corrected_image.cols() || raw_image.rows() != corrected_image.rows());

    const int32_t rows = corrected_image.rows();
    const int32_t cols = corrected_image.cols();
    scale = std::max(1e0f, scale);

    Vec2 distort_uv = Vec2::Zero();
    for (int32_t u = 0; u < cols; ++u) {
        for (int32_t v = 0; v < rows; ++v) {
            Vec2 undistort_uv = Vec2(u, v);
            const Vec2 mid_uv = Vec2(cx_, cy_);
            undistort_uv = scale * (undistort_uv - mid_uv) + mid_uv;

            DistortOnImagePlane(undistort_uv, distort_uv);

            float pixel_value = 0.0f;
            if (raw_image.GetPixelValue(distort_uv.y(), distort_uv.x(), &pixel_value)) {
                corrected_image.SetPixelValue(v, u, static_cast<uint8_t>(pixel_value));
            } else {
                corrected_image.SetPixelValue(v, u, 0);
            }
        }
    }

    return true;
}

void CameraPinhole::SetIntrinsicParameter(float fx, float fy, float cx, float cy) {
    fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;
}

}  // namespace sensor_model
