#include "camera_basic.h"

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

void CameraBasic::SetMatrixK(float fx, float fy, float cx, float cy) {
	fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;
}

}