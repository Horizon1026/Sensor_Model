#include "camera_omni.h"

#include "slam_operations.h"

#include "algorithm"
#include "cmath"
#include "limits"
#include "stdexcept"

namespace sensor_model {

CameraOmni::CameraOmni(float fx, float fy, float cx, float cy, float xi): CameraOmni(fx, fy, cx, cy, xi, 0, 0) {}

CameraOmni::CameraOmni(float fx, float fy, float cx, float cy, float xi, int32_t image_rows, int32_t image_cols):
    fx_(fx), fy_(fy), cx_(cx), cy_(cy), image_rows_(image_rows > 0 ? image_rows : static_cast<int32_t>(cy * 2.0f)),
    image_cols_(image_cols > 0 ? image_cols : static_cast<int32_t>(cx * 2.0f)) {
    SetIntrinsicParameter(fx, fy, cx, cy, xi);
}

// Lift 2d point in normalized plane to other coordinate systems.
void CameraOmni::LiftFromNormalizedPlaneToUnitSphere(const Vec2 norm_xy, Vec3 &sphere_xyz) const {
    const double r2 = norm_xy.cast<double>().squaredNorm();
    const double xi = xi_;
    const double radicand = 1.0 + (1.0 - xi * xi) * r2;
    if (!norm_xy.allFinite() || radicand < 0.0) {
        sphere_xyz.setConstant(std::numeric_limits<float>::quiet_NaN());
        return;
    }
    const double lambda = (xi + std::sqrt(radicand)) / (1.0 + r2);
    sphere_xyz << lambda * norm_xy.x(), lambda * norm_xy.y(), lambda - xi;
    // The boundary is singular for xi > 1 and is excluded from projection.
    if (xi > 1.0 && radicand <= 0.0) {
        sphere_xyz.setConstant(std::numeric_limits<float>::quiet_NaN());
    }
}
void CameraOmni::LiftFromNormalizedPlaneToImagePlane(const Vec2 norm_xy, Vec2 &pixel_uv) const {
    pixel_uv(0) = fx_ * norm_xy(0) + cx_;
    pixel_uv(1) = fy_ * norm_xy(1) + cy_;
}
void CameraOmni::LiftFromNormalizedPlaneToBearingVector(const Vec2 norm_xy, Vec3 &bearing_vector) const {
    LiftFromNormalizedPlaneToUnitSphere(norm_xy, bearing_vector);
}

// Lift 3d point in unit sphere to other coordinate systems.
void CameraOmni::LiftFromUnitSphereToNormalizedPlane(const Vec3 sphere_xyz, Vec2 &norm_xy) const { LiftFromCameraFrameToNormalizedPlane(sphere_xyz, norm_xy); }
void CameraOmni::LiftFromUnitSphereToBearingVector(const Vec3 sphere_xyz, Vec3 &bearing_vector) const {
    Vec2 norm_xy;
    LiftFromUnitSphereToNormalizedPlane(sphere_xyz, norm_xy);
    LiftFromNormalizedPlaneToBearingVector(norm_xy, bearing_vector);
}
void CameraOmni::LiftFromUnitSphereToImagePlane(const Vec3 sphere_xyz, Vec2 &pixel_uv) const {
    Vec2 norm_xy = Vec2::Zero();
    LiftFromUnitSphereToNormalizedPlane(sphere_xyz, norm_xy);
    LiftFromNormalizedPlaneToImagePlane(norm_xy, pixel_uv);
}

// Lift 3d point in bearing vector to other coordinate systems.
void CameraOmni::LiftFromBearingVectorToNormalizedPlane(const Vec3 bearing_vector, Vec2 &norm_xy) const {
    LiftFromCameraFrameToNormalizedPlane(bearing_vector, norm_xy);
}
void CameraOmni::LiftFromBearingVectorToUnitSphere(const Vec3 bearing_vector, Vec3 &sphere_xyz) const {
    Vec2 norm_xy = Vec2::Zero();
    LiftFromBearingVectorToNormalizedPlane(bearing_vector, norm_xy);
    LiftFromNormalizedPlaneToUnitSphere(norm_xy, sphere_xyz);
}
void CameraOmni::LiftFromBearingVectorToImagePlane(const Vec3 bearing_vector, Vec2 &pixel_uv) const {
    Vec2 norm_xy = Vec2::Zero();
    LiftFromBearingVectorToNormalizedPlane(bearing_vector, norm_xy);
    LiftFromNormalizedPlaneToImagePlane(norm_xy, pixel_uv);
}

// Lift 2d point in image plane back on normalized plane.
void CameraOmni::LiftFromImagePlaneToNormalizedPlane(const Vec2 pixel_uv, Vec2 &norm_xy) const {
    if (!pixel_uv.allFinite() || !std::isfinite(fx_) || !std::isfinite(fy_) || fx_ == 0.0f || fy_ == 0.0f) {
        norm_xy.setConstant(std::numeric_limits<float>::quiet_NaN());
        return;
    }
    norm_xy(0) = (pixel_uv(0) - cx_) / fx_;
    norm_xy(1) = (pixel_uv(1) - cy_) / fy_;
}
void CameraOmni::LiftFromImagePlaneToBearingVector(const Vec2 pixel_uv, Vec3 &bearing_vector) const {
    Vec2 norm_xy = Vec2::Zero();
    LiftFromImagePlaneToNormalizedPlane(pixel_uv, norm_xy);
    LiftFromNormalizedPlaneToBearingVector(norm_xy, bearing_vector);
}
void CameraOmni::LiftFromImagePlaneToUnitSphere(const Vec2 pixel_uv, Vec3 &sphere_xyz) const {
    Vec2 norm_xy = Vec2::Zero();
    LiftFromImagePlaneToNormalizedPlane(pixel_uv, norm_xy);
    LiftFromNormalizedPlaneToUnitSphere(norm_xy, sphere_xyz);
}

// Lift 3d point in camera frame on normalized plane.
void CameraOmni::LiftFromCameraFrameToNormalizedPlane(const Vec3 p_c, Vec2 &norm_xy) const {
    const double length = p_c.cast<double>().norm();
    const double z = length > 0.0 ? p_c.z() / length : -1.0;
    const double limit = xi_ <= 1.0f ? -static_cast<double>(xi_) : -1.0 / xi_;
    if (!p_c.allFinite() || length == 0.0 || z <= limit) {
        norm_xy.setConstant(std::numeric_limits<float>::quiet_NaN());
        return;
    }
    const double denominator = p_c.z() + xi_ * length;
    norm_xy = (p_c.head<2>().cast<double>() / denominator).cast<float>();
}

// Distort on normalized plane.
bool CameraOmni::DistortOnNormalizedPlane(const Vec2 undistort_xy, Vec2 &distort_xy) const {
    // No lens distortion in the base omni model.
    distort_xy = undistort_xy;
    return distort_xy.allFinite();
}

// Undistort on normalized plane.
bool CameraOmni::UndistortOnNormalizedPlane(const Vec2 distort_xy, Vec2 &undistort_xy) const {
    // No lens distortion in the base omni model.
    undistort_xy = distort_xy;
    return undistort_xy.allFinite();
}

// Lift 3d point in image plane on normalized plane, and do undistortion.
bool CameraOmni::LiftFromRawImagePlaneToUndistortedNormalizedPlane(const Vec2 pixel_uv, Vec2 &undistort_xy) const {
    Vec2 distort_xy = Vec2::Zero();
    LiftFromImagePlaneToNormalizedPlane(pixel_uv, distort_xy);
    RETURN_FALSE_IF_FALSE(UndistortOnNormalizedPlane(distort_xy, undistort_xy));
    Vec3 bearing;
    LiftFromNormalizedPlaneToBearingVector(undistort_xy, bearing);
    return bearing.allFinite();
}

bool CameraOmni::DistortOnImagePlane(const Vec2 undistort_uv, Vec2 &distort_uv) const {
    Vec2 undistort_xy;
    LiftFromImagePlaneToNormalizedPlane(undistort_uv, undistort_xy);
    Vec2 distort_xy = Vec2::Zero();
    RETURN_FALSE_IF_FALSE(DistortOnNormalizedPlane(undistort_xy, distort_xy));
    LiftFromNormalizedPlaneToImagePlane(distort_xy, distort_uv);
    return distort_uv.allFinite();
}

bool CameraOmni::UndistortOnImagePlane(const Vec2 distort_uv, Vec2 &undistort_uv) const {
    Vec2 distort_xy;
    LiftFromImagePlaneToNormalizedPlane(distort_uv, distort_xy);
    Vec2 undistort_xy = Vec2::Zero();
    RETURN_FALSE_IF_FALSE(UndistortOnNormalizedPlane(distort_xy, undistort_xy));
    LiftFromNormalizedPlaneToImagePlane(undistort_xy, undistort_uv);
    return undistort_uv.allFinite();
}

// Undistort image.
bool CameraOmni::CorrectDistortedImage(const GrayImage &raw_image, GrayImage &corrected_image, float scale) const {
    RETURN_FALSE_IF(raw_image.data() == nullptr || corrected_image.data() == nullptr);
    RETURN_FALSE_IF(raw_image.cols() != corrected_image.cols() || raw_image.rows() != corrected_image.rows());

    RETURN_FALSE_IF(raw_image.data() == corrected_image.data());
    RETURN_FALSE_IF(!std::isfinite(scale) || !std::isfinite(fx_) || !std::isfinite(fy_) || !std::isfinite(cx_) || !std::isfinite(cy_) || fx_ == 0.0f ||
                    fy_ == 0.0f);

    const int32_t rows = corrected_image.rows();
    const int32_t cols = corrected_image.cols();
    scale = std::max(1e0f, scale);

    Vec2 distort_uv = Vec2::Zero();
    for (int32_t u = 0; u < cols; ++u) {
        for (int32_t v = 0; v < rows; ++v) {
            Vec2 undistort_uv = Vec2(u, v);
            const Vec2 mid_uv = Vec2(cx_, cy_);
            undistort_uv = scale * (undistort_uv - mid_uv) + mid_uv;

            Vec3 bearing;
            LiftFromImagePlaneToBearingVector(undistort_uv, bearing);
            if (!bearing.allFinite() || !DistortOnImagePlane(undistort_uv, distort_uv)) {
                corrected_image.SetPixelValue(v, u, 0);
                continue;
            }

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

void CameraOmni::SetImageSize(int32_t image_rows, int32_t image_cols) {
    image_rows_ = image_rows;
    image_cols_ = image_cols;
}

void CameraOmni::SetIntrinsicParameter(float fx, float fy, float cx, float cy, float xi) {
    if (!std::isfinite(xi) || xi < 0.0f) {
        throw std::invalid_argument("CameraOmni xi must be finite and nonnegative");
    }
    xi_ = xi;
    fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;
}

}  // namespace sensor_model
