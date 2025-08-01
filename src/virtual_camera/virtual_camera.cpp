#include "virtual_camera.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "slam_basic_math.h"

namespace SENSOR_MODEL {

bool VirtualCamera::GenerateMaphex(const Quat &q_wc) {
    return GenerateMaphex(q_wc, options_.kTargetQwc);
}

bool VirtualCamera::GenerateMaphex(const Quat &q_wc, const Quat &target_q_wc) {
    RETURN_FALSE_IF(raw_camera_model_ == nullptr);
    RETURN_FALSE_IF(options_.kVirtualImageRows <= 0);
    RETURN_FALSE_IF(options_.kVirtualImageCols <= 0);
    virtual_camera_model_ = std::make_unique<CameraBasic>(options_.kVirtualCameraFocusLength,
                                                          options_.kVirtualCameraFocusLength,
                                                          options_.kVirtualImageCols * 0.5f,
                                                          options_.kVirtualImageRows * 0.5f);

    const Mat3 R_wc0 = target_q_wc.toRotationMatrix();
    const Mat3 R_wc = q_wc.toRotationMatrix();
    H_cc0_ = R_wc.transpose() * R_wc0;
    maphex_row_.setZero(options_.kVirtualImageRows, options_.kVirtualImageCols);
    maphex_col_.setZero(options_.kVirtualImageRows, options_.kVirtualImageCols);

    for (int32_t row = 0; row < options_.kVirtualImageRows; ++row) {
        for (int32_t col = 0; col < options_.kVirtualImageCols; ++col) {
            const Vec2 virtual_pixel_uv = Vec2(col, row);
            Vec2 virtual_norm_xy = Vec2::Zero();
            virtual_camera_model_->LiftFromImagePlaneToNormalizedPlane(virtual_pixel_uv, virtual_norm_xy);
            Vec3 raw_undistort_norm_xy1 = H_cc0_ * Vec3(virtual_norm_xy.x(), virtual_norm_xy.y(), 1.0f);
            RETURN_FALSE_IF(raw_undistort_norm_xy1.z() < kZeroFloat);
            raw_undistort_norm_xy1 /= raw_undistort_norm_xy1.z();
            Vec2 raw_distort_norm_xy = Vec2::Zero();
            raw_camera_model_->DistortOnNormalizedPlane(raw_undistort_norm_xy1.head<2>(), raw_distort_norm_xy);
            Vec2 raw_distort_pixel_uv = Vec2::Zero();
            raw_camera_model_->LiftFromNormalizedPlaneToImagePlane(raw_distort_norm_xy, raw_distort_pixel_uv);
            maphex_col_(row, col) = raw_distort_pixel_uv.x();
            maphex_row_(row, col) = raw_distort_pixel_uv.y();
        }
    }

    return true;
}

bool VirtualCamera::RemapVirtualCameraImage(const GrayImage &raw_image) {
    RETURN_FALSE_IF(raw_image.data() == nullptr);
    RETURN_FALSE_IF(raw_image.cols() == 0 || raw_image.rows() == 0);

    virtual_camera_image_.setZero(options_.kVirtualImageRows, options_.kVirtualImageCols);
    virtual_camera_mask_.setZero(options_.kVirtualImageRows, options_.kVirtualImageCols);

    for (int32_t row = 0; row < options_.kVirtualImageRows; ++row) {
        for (int32_t col = 0; col < options_.kVirtualImageCols; ++col) {
            float pixel_value = 0.0f;
            CONTINUE_IF(!raw_image.GetPixelValue(maphex_row_(row, col), maphex_col_(row, col), &pixel_value));
            virtual_camera_image_(row, col) = static_cast<uint8_t>(static_cast<int32_t>(pixel_value));
            virtual_camera_mask_(row, col) = 255;
        }
    }

    return true;
}

}
