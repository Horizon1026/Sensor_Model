#include "camera_basic.h"
#include "slam_operations.h"

namespace SensorModel {

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

void CameraBasic::SetMatrixK(float fx, float fy, float cx, float cy) {
	fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;
}

}