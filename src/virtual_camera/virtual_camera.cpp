#include "virtual_camera.h"

#include "slam_basic_math.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"

namespace sensor_model {

namespace {
bool RotateNormalizedPoint(const BasicCameraModel &source, const BasicCameraModel &target,
                           const Mat3 &rotation, const Vec2 &source_xy, Vec2 &target_xy) {
    Vec3 source_ray;
    source.LiftFromNormalizedPlaneToBearingVector(source_xy, source_ray);
    const Vec3 target_ray = rotation * source_ray;
    if (!target_ray.allFinite() || target_ray.squaredNorm() == 0.0f) {
        return false;
    }
    target.LiftFromCameraFrameToNormalizedPlane(target_ray, target_xy);
    if (!target_xy.allFinite()) {
        return false;
    }
    // Pinhole returns zero for invisible rays; ensure projection preserves direction.
    Vec3 recovered_ray;
    target.LiftFromNormalizedPlaneToBearingVector(target_xy, recovered_ray);
    return recovered_ray.allFinite() && recovered_ray.dot(target_ray) > 0.0f;
}
}  // namespace


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
    H_virtual_to_real_ = q_rv.toRotationMatrix();
    H_real_to_virtual_ = q_rv.inverse().toRotationMatrix();
    maphex_row_.setZero(virtual_camera_model_->image_rows(), virtual_camera_model_->image_cols());
    maphex_col_.setZero(virtual_camera_model_->image_rows(), virtual_camera_model_->image_cols());
    for (int32_t row = 0; row < virtual_camera_model_->image_rows(); ++row) {
        for (int32_t col = 0; col < virtual_camera_model_->image_cols(); ++col) {
            const Vec2 virtual_distort_pixel_uv = Vec2(col, row);
            Vec2 real_distort_pixel_uv = Vec2::Zero();
            if (!RemapPixelUvFromVirtualCameraToRealCamera(virtual_distort_pixel_uv, real_distort_pixel_uv)) {
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

bool VirtualCamera::RemapPixelUvFromVirtualCameraToRealCamera(const Vec2 &virtual_distort_pixel_uv, Vec2 &real_distort_pixel_uv) {
    RETURN_FALSE_IF(real_camera_model_ == nullptr || virtual_camera_model_ == nullptr);

    Vec2 virtual_norm_xy, real_norm_xy, real_distort_norm_xy;
    RETURN_FALSE_IF(!virtual_camera_model_->LiftFromRawImagePlaneToUndistortedNormalizedPlane(virtual_distort_pixel_uv, virtual_norm_xy));
    RETURN_FALSE_IF(!RotateNormalizedPoint(*virtual_camera_model_, *real_camera_model_, H_virtual_to_real_, virtual_norm_xy, real_norm_xy));
    RETURN_FALSE_IF(!real_camera_model_->DistortOnNormalizedPlane(real_norm_xy, real_distort_norm_xy));
    real_camera_model_->LiftFromNormalizedPlaneToImagePlane(real_distort_norm_xy, real_distort_pixel_uv);
    return real_distort_pixel_uv.allFinite();
}

bool VirtualCamera::RemapPixelUvFromRealCameraToVirtualCamera(const Vec2 &real_distort_pixel_uv, Vec2 &virtual_distort_pixel_uv) {
    RETURN_FALSE_IF(real_camera_model_ == nullptr || virtual_camera_model_ == nullptr);

    Vec2 real_norm_xy, virtual_norm_xy, virtual_distort_norm_xy;
    RETURN_FALSE_IF(!real_camera_model_->LiftFromRawImagePlaneToUndistortedNormalizedPlane(real_distort_pixel_uv, real_norm_xy));
    RETURN_FALSE_IF(!RotateNormalizedPoint(*real_camera_model_, *virtual_camera_model_, H_real_to_virtual_, real_norm_xy, virtual_norm_xy));
    RETURN_FALSE_IF(!virtual_camera_model_->DistortOnNormalizedPlane(virtual_norm_xy, virtual_distort_norm_xy));
    virtual_camera_model_->LiftFromNormalizedPlaneToImagePlane(virtual_distort_norm_xy, virtual_distort_pixel_uv);
    return virtual_distort_pixel_uv.allFinite();
}

bool VirtualCamera::RemapUndistortedNormXyFromVirtualCameraToRealCamera(const Vec2 &virtual_undistorted_norm_xy, Vec2 &real_undistorted_norm_xy) {
    RETURN_FALSE_IF(real_camera_model_ == nullptr || virtual_camera_model_ == nullptr);
    return RotateNormalizedPoint(*virtual_camera_model_, *real_camera_model_, H_virtual_to_real_, virtual_undistorted_norm_xy, real_undistorted_norm_xy);
}

bool VirtualCamera::RemapUndistortedNormXyFromRealCameraToVirtualCamera(const Vec2 &real_undistorted_norm_xy, Vec2 &virtual_undistorted_norm_xy) {
    RETURN_FALSE_IF(real_camera_model_ == nullptr || virtual_camera_model_ == nullptr);
    return RotateNormalizedPoint(*real_camera_model_, *virtual_camera_model_, H_real_to_virtual_, real_undistorted_norm_xy, virtual_undistorted_norm_xy);
}

Vec2 VirtualCamera::GetVirtualCameraFov() const {
    if (virtual_camera_model_ == nullptr) {
        return Vec2::Zero();
    }

    Vec3 horizontal_ray, vertical_ray;
    virtual_camera_model_->LiftFromImagePlaneToBearingVector(Vec2(0, virtual_camera_model_->cy()), horizontal_ray);
    virtual_camera_model_->LiftFromImagePlaneToBearingVector(Vec2(virtual_camera_model_->cx(), 0), vertical_ray);
    return kRadToDeg * 2.0f * Vec2(std::atan2(horizontal_ray.head<2>().norm(), horizontal_ray.z()),
                                  std::atan2(vertical_ray.head<2>().norm(), vertical_ray.z()));
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
