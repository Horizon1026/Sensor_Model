#include "virtual_camera.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"

namespace sensor_model {

bool VirtualCamera::CreateDefaultVirtualCameraModel() {
    RETURN_FALSE_IF(options_.kDefaultVirtualCameraImageRows <= 0);
    RETURN_FALSE_IF(options_.kDefaultVirtualCameraImageCols <= 0);
    virtual_camera_model_ = std::make_unique<CameraPinhole>(options_.kDefaultVirtualCameraFocalLength, options_.kDefaultVirtualCameraFocalLength,
                                                            options_.kDefaultVirtualCameraImageCols * 0.5f, options_.kDefaultVirtualCameraImageRows * 0.5f);
    return true;
}

bool VirtualCamera::GenerateMaphex() { return GenerateMaphex(options_.kQrv); }

bool VirtualCamera::GenerateMaphex(const Quat &q_rv) {
    RETURN_FALSE_IF(real_camera_model_ == nullptr);

    // Auto generate virtual camera model if not configured.
    if (virtual_camera_model_ == nullptr) {
        RETURN_FALSE_IF(!CreateDefaultVirtualCameraModel());
        options_.kQrv = q_rv.normalized();
    }

    // Generate maphex.
    H_cc0_ = q_rv.toRotationMatrix();
    maphex_row_.setZero(virtual_camera_model_->image_rows(), virtual_camera_model_->image_cols());
    maphex_col_.setZero(virtual_camera_model_->image_rows(), virtual_camera_model_->image_cols());
    for (int32_t row = 0; row < virtual_camera_model_->image_rows(); ++row) {
        for (int32_t col = 0; col < virtual_camera_model_->image_cols(); ++col) {
            const Vec2 virtual_distort_pixel_uv = Vec2(col, row);
            Vec2 real_distort_pixel_uv = Vec2::Zero();
            if (!RemapPixelUvFromVirtualCameraToRawCamera(virtual_distort_pixel_uv, real_distort_pixel_uv)) {
                maphex_col_(row, col) = -1;
                maphex_row_(row, col) = -1;
                continue;
            }
            maphex_col_(row, col) = real_distort_pixel_uv.x();
            maphex_row_(row, col) = real_distort_pixel_uv.y();
        }
    }

    return true;
}

bool VirtualCamera::RemapVirtualCameraImage(const GrayImage &raw_image) {
    RETURN_FALSE_IF(virtual_camera_model_ == nullptr);
    RETURN_FALSE_IF(raw_image.data() == nullptr);
    RETURN_FALSE_IF(raw_image.cols() == 0 || raw_image.rows() == 0);

    const int32_t image_rows = virtual_camera_model_->image_rows();
    const int32_t image_cols = virtual_camera_model_->image_cols();
    virtual_camera_image_.setZero(image_rows, image_cols);
    virtual_camera_mask_.setZero(image_rows, image_cols);

    for (int32_t row = 0; row < image_rows; ++row) {
        for (int32_t col = 0; col < image_cols; ++col) {
            float pixel_value = 0.0f;
            CONTINUE_IF(!raw_image.GetPixelValue(maphex_row_(row, col), maphex_col_(row, col), &pixel_value));
            virtual_camera_image_(row, col) = static_cast<uint8_t>(static_cast<int32_t>(pixel_value));
            virtual_camera_mask_(row, col) = 255;
        }
    }

    return true;
}

bool VirtualCamera::RemapPixelUvFromVirtualCameraToRawCamera(const Vec2 &virtual_distort_pixel_uv, Vec2 &real_distort_pixel_uv) {
    RETURN_FALSE_IF(real_camera_model_ == nullptr || virtual_camera_model_ == nullptr);

    Vec2 virtual_norm_xy = Vec2::Zero();
    virtual_camera_model_->LiftFromRawImagePlaneToUndistortedNormalizedPlane(virtual_distort_pixel_uv, virtual_norm_xy);
    Vec3 real_undistort_norm_xy1 = H_cc0_ * Vec3(virtual_norm_xy.x(), virtual_norm_xy.y(), 1.0f);
    RETURN_FALSE_IF(real_undistort_norm_xy1.z() < kZeroFloat);
    real_undistort_norm_xy1 /= real_undistort_norm_xy1.z();
    Vec2 real_distort_norm_xy = Vec2::Zero();
    real_camera_model_->DistortOnNormalizedPlane(real_undistort_norm_xy1.head<2>(), real_distort_norm_xy);
    real_camera_model_->LiftFromNormalizedPlaneToImagePlane(real_distort_norm_xy, real_distort_pixel_uv);
    return true;
}

bool VirtualCamera::RemapPixelUvFromRawCameraToVirtualCamera(const Vec2 &real_distort_pixel_uv, Vec2 &virtual_distort_pixel_uv) {
    RETURN_FALSE_IF(real_camera_model_ == nullptr || virtual_camera_model_ == nullptr);

    Vec2 real_undistort_norm_xy = Vec2::Zero();
    real_camera_model_->LiftFromRawImagePlaneToUndistortedNormalizedPlane(real_distort_pixel_uv, real_undistort_norm_xy);
    Vec3 virtual_undistort_norm_xy1 = H_cc0_.transpose() * Vec3(real_undistort_norm_xy.x(), real_undistort_norm_xy.y(), 1.0f);
    RETURN_FALSE_IF(virtual_undistort_norm_xy1.z() < kZeroFloat);
    virtual_undistort_norm_xy1 / virtual_undistort_norm_xy1.z();
    Vec2 virtual_distort_norm_xy = Vec2::Zero();
    virtual_camera_model_->DistortOnNormalizedPlane(virtual_undistort_norm_xy1.head<2>(), virtual_distort_norm_xy);
    virtual_camera_model_->LiftFromImagePlaneToNormalizedPlane(virtual_distort_norm_xy, virtual_distort_pixel_uv);
    return true;
}

Vec2 VirtualCamera::GetVirtualCameraFov() const {
    if (virtual_camera_model_ == nullptr) {
        return Vec2::Zero();
    }

    return kRadToDeg * 2.0f *
           Vec2(std::atan2(virtual_camera_model_->cx(), virtual_camera_model_->fx()), std::atan2(virtual_camera_model_->cy(), virtual_camera_model_->fy()));
}

int32_t VirtualCamera::GetVirtualCameraImageRows() const {
    if (virtual_camera_model_ == nullptr) {
        return 0;
    }
    return virtual_camera_model_->image_rows();
}

int32_t VirtualCamera::GetVirtualCameraImageCols() const {
    if (virtual_camera_model_ == nullptr) {
        return 0;
    }
    return virtual_camera_model_->image_cols();
}

}  // namespace sensor_model
