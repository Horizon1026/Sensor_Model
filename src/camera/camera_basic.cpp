#include "camera_basic.h"
#include "slam_operations.h"
#include "math_kinematics.h"

namespace SENSOR_MODEL {

// Lift 3d point in camera frame on normalized plane.
void CameraBasic::LiftToNormalizedPlane(const Vec3 p_c, Vec2 &norm_xy) {
    if (p_c.z() < kZero) {
        norm_xy.setZero();
    } else {
        norm_xy.x() = p_c.x() / p_c.z();
        norm_xy.y() = p_c.y() / p_c.z();
    }
}

// Lift 2d point in normalized plane on image plane.
void CameraBasic::LiftToImagePlane(const Vec2 norm_xy, Vec2 &pixel_uv) {
	pixel_uv(0) = fx_ * norm_xy(0) + cx_;
    pixel_uv(1) = fy_ * norm_xy(1) + cy_;
}

// Lift 2d point in image plane back on normalized plane.
void CameraBasic::LiftBackToNormalizedPlane(const Vec2 pixel_uv, Vec2 &norm_xy) {
	norm_xy(0) = (pixel_uv(0) - cx_) / fx_;
    norm_xy(1) = (pixel_uv(1) - cy_) / fy_;
}

// Lift 3d point in camera frame on normalized plane, and do undistortion.
bool CameraBasic::LiftToNormalizedPlaneAndUndistort(const Vec2 pixel_uv, Vec2 &undistort_xy) {
    Vec2 distort_xy;
    LiftBackToNormalizedPlane(pixel_uv, distort_xy);
    return UndistortOnNormalizedPlane(distort_xy, undistort_xy);
}

bool CameraBasic::DistortOnImagePlane(const Vec2 undistort_uv, Vec2 &distort_uv) {
    Vec2 undistort_xy = Vec2((undistort_uv(0) - cx()) / fx(),
                             (undistort_uv(1) - cy()) / fy());

    Vec2 distort_xy;
    RETURN_FALSE_IF_FALSE(DistortOnNormalizedPlane(undistort_xy, distort_xy));

    distort_uv(0) = distort_xy(0) * fx() + cx();
    distort_uv(1) = distort_xy(1) * fy() + cy();

    return true;
}

bool CameraBasic::UndistortOnImagePlane(const Vec2 distort_uv, Vec2 &undistort_uv) {
    Vec2 distort_xy = Vec2((distort_uv(0) - cx()) / fx(),
                           (distort_uv(1) - cy()) / fy());

    Vec2 undistort_xy;
    RETURN_FALSE_IF_FALSE(UndistortOnNormalizedPlane(distort_xy, undistort_xy));

    undistort_uv(0) = undistort_xy(0) * fx() + cx();
    undistort_uv(1) = undistort_xy(1) * fy() + cy();

    return true;
}

// Undistort image.
bool CameraBasic::CorrectDistortedImage(const Image &raw_image, Image &corrected_image, float scale) {
    if (raw_image.data() == nullptr || corrected_image.data() == nullptr) {
        return false;
    }

    if (raw_image.cols() != corrected_image.cols() || raw_image.rows() != corrected_image.rows()) {
        return false;
    }

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

void CameraBasic::SetIntrinsicParameter(float fx, float fy, float cx, float cy) {
	fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;
}

}
