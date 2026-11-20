#include "virtual_camera.h"

#include "slam_basic_math.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"

#include "algorithm"
#include "limits"

namespace sensor_model {

bool VirtualCamera::CreateDefaultVirtualCameraModel() {
    RETURN_FALSE_IF(options_.kDefaultVirtualCameraImageRows <= 0);
    RETURN_FALSE_IF(options_.kDefaultVirtualCameraImageCols <= 0);
    virtual_camera_model_ =
        std::make_unique<CameraPinhole<float>>(options_.kDefaultVirtualCameraFocalLength, options_.kDefaultVirtualCameraFocalLength,
                                               options_.kDefaultVirtualCameraImageCols * 0.5f, options_.kDefaultVirtualCameraImageRows * 0.5f);
    return true;
}

bool VirtualCamera::GenerateMaphex() { return GenerateMaphex(options_.kQrv); }

bool VirtualCamera::GenerateMaphex(const Quat &q_rv) {
    RETURN_FALSE_IF(real_camera_model_ == nullptr);
    const Quat rotation = q_rv.normalized();

    // Auto generate virtual camera model if not configured.
    if (virtual_camera_model_ == nullptr) {
        RETURN_FALSE_IF(!CreateDefaultVirtualCameraModel());
    }

    // Generate maphex.
    options_.kQrv = rotation;
    H_virtual_to_real_ = rotation.toRotationMatrix();
    H_real_to_virtual_ = H_virtual_to_real_.transpose();
    GenerateBearingMap();
    maphex_row_.resize(virtual_camera_model_->image_rows(), virtual_camera_model_->image_cols());
    maphex_col_.resize(virtual_camera_model_->image_rows(), virtual_camera_model_->image_cols());
    GenerateMaphexSimd();
    return true;
}

void VirtualCamera::GenerateBearingMap() {
    const auto &camera = *virtual_camera_model_;
    const auto &camera_options = camera.options();
    const std::array<int64_t, 4> integer_parameters {
        camera.image_rows(),
        camera.image_cols(),
        static_cast<int64_t>(camera_options.kUndistortMethod),
        camera_options.kMaxIterationForUndistortion,
    };
    std::vector<float> parameters {
        camera.fx(), camera.fy(), camera.cx(), camera.cy(), camera_options.kMaxIterateStepLengthForUndistortion,
    };
    const auto *omni_camera = dynamic_cast<const CameraOmni<float> *>(&camera);
    if (omni_camera != nullptr) {
        parameters.emplace_back(omni_camera->xi());
    }
    std::vector<float> distortion_parameters {};
    camera.GetDistortionParameter(distortion_parameters);
    parameters.insert(parameters.end(), distortion_parameters.begin(), distortion_parameters.end());
    const std::string camera_name = camera.CameraModelName();
    RETURN_IF(bearing_map_camera_ptr_ == &camera && bearing_map_camera_name_ == camera_name && bearing_map_integer_parameters_ == integer_parameters &&
              bearing_map_parameters_ == parameters);

    for (Mat &bearing_map: bearing_map_xyz_) {
        bearing_map.resize(camera.image_rows(), camera.image_cols());
    }
    bearing_map_valid_.resize(camera.image_rows(), camera.image_cols());
    for (int32_t row = 0; row < camera.image_rows(); ++row) {
        for (int32_t col = 0; col < camera.image_cols(); ++col) {
            Vec3 bearing_xyz = Vec3::Zero();
            bearing_map_valid_(row, col) = camera.LiftFromRawImagePlaneToBearingVector(Vec2(col, row), bearing_xyz);
            for (uint32_t axis = 0; axis < 3; ++axis) {
                bearing_map_xyz_[axis](row, col) = bearing_xyz[axis];
            }
        }
    }
    bearing_map_camera_ptr_ = &camera;
    bearing_map_camera_name_ = camera_name;
    bearing_map_integer_parameters_ = integer_parameters;
    bearing_map_parameters_ = std::move(parameters);
}

void VirtualCamera::GenerateMaphexCpu() {
    for (int32_t row = 0; row < virtual_camera_model_->image_rows(); ++row) {
        for (int32_t col = 0; col < virtual_camera_model_->image_cols(); ++col) {
            maphex_col_(row, col) = std::numeric_limits<float>::quiet_NaN();
            maphex_row_(row, col) = std::numeric_limits<float>::quiet_NaN();
            CONTINUE_IF(!bearing_map_valid_(row, col));
            const Vec3 virtual_bearing_xyz(bearing_map_xyz_[0](row, col), bearing_map_xyz_[1](row, col), bearing_map_xyz_[2](row, col));
            const Vec3 real_bearing_xyz = H_virtual_to_real_ * virtual_bearing_xyz;
            Vec2 real_distort_pixel_uv = Vec2::Zero();
            CONTINUE_IF(!real_camera_model_->LiftFromBearingVectorToRawImagePlane(real_bearing_xyz, real_distort_pixel_uv));
            maphex_col_(row, col) = real_distort_pixel_uv.x();
            maphex_row_(row, col) = real_distort_pixel_uv.y();
        }
    }
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
            const float pixel_row = maphex_row_(row, col);
            const float pixel_col = maphex_col_(row, col);
            CONTINUE_IF(!std::isfinite(pixel_row) || !std::isfinite(pixel_col));
            float pixel_value = 0.0f;
            const bool is_valid = raw_image.GetPixelValue(pixel_row, pixel_col, &pixel_value);
            if (!is_valid) {
                // Replicate source borders while preserving interpolation along the edge.
                const float clamped_row = std::clamp(pixel_row, 0.0f, static_cast<float>(raw_image.rows() - 1));
                const float clamped_col = std::clamp(pixel_col, 0.0f, static_cast<float>(raw_image.cols() - 1));
                const int32_t row_0 = static_cast<int32_t>(clamped_row);
                const int32_t col_0 = static_cast<int32_t>(clamped_col);
                const int32_t row_1 = std::min(row_0 + 1, raw_image.rows() - 1);
                const int32_t col_1 = std::min(col_0 + 1, raw_image.cols() - 1);
                const float row_weight = clamped_row - row_0;
                const float col_weight = clamped_col - col_0;
                pixel_value =
                    (1.0f - row_weight) *
                        ((1.0f - col_weight) * raw_image.GetPixelValueNoCheck(row_0, col_0) + col_weight * raw_image.GetPixelValueNoCheck(row_0, col_1)) +
                    row_weight *
                        ((1.0f - col_weight) * raw_image.GetPixelValueNoCheck(row_1, col_0) + col_weight * raw_image.GetPixelValueNoCheck(row_1, col_1));
            }
            virtual_camera_image_(row, col) = static_cast<uint8_t>(static_cast<int32_t>(pixel_value));
            virtual_camera_mask_(row, col) = is_valid ? 255 : 0;
        }
    }

    return true;
}

bool VirtualCamera::RemapPixelUvFromVirtualCameraToRealCamera(const Vec2 &virtual_distort_pixel_uv, Vec2 &real_distort_pixel_uv) {
    RETURN_FALSE_IF(real_camera_model_ == nullptr || virtual_camera_model_ == nullptr);

    Vec3 virtual_bearing_xyz = Vec3::Zero();
    Vec3 real_bearing_xyz = Vec3::Zero();
    RETURN_FALSE_IF(!virtual_camera_model_->LiftFromRawImagePlaneToBearingVector(virtual_distort_pixel_uv, virtual_bearing_xyz));
    RemapBearingVectorFromVirtualCameraToRealCamera(virtual_bearing_xyz, real_bearing_xyz);
    return real_camera_model_->LiftFromBearingVectorToRawImagePlane(real_bearing_xyz, real_distort_pixel_uv);
}

bool VirtualCamera::RemapPixelUvFromRealCameraToVirtualCamera(const Vec2 &real_distort_pixel_uv, Vec2 &virtual_distort_pixel_uv) {
    RETURN_FALSE_IF(real_camera_model_ == nullptr || virtual_camera_model_ == nullptr);

    Vec3 real_bearing_xyz = Vec3::Zero();
    Vec3 virtual_bearing_xyz = Vec3::Zero();
    RETURN_FALSE_IF(!real_camera_model_->LiftFromRawImagePlaneToBearingVector(real_distort_pixel_uv, real_bearing_xyz));
    RemapBearingVectorFromRealCameraToVirtualCamera(real_bearing_xyz, virtual_bearing_xyz);
    return virtual_camera_model_->LiftFromBearingVectorToRawImagePlane(virtual_bearing_xyz, virtual_distort_pixel_uv);
}

void VirtualCamera::RemapBearingVectorFromVirtualCameraToRealCamera(const Vec3 &virtual_bearing_xyz, Vec3 &real_bearing_xyz) const {
    real_bearing_xyz = H_virtual_to_real_ * virtual_bearing_xyz;
}

void VirtualCamera::RemapBearingVectorFromRealCameraToVirtualCamera(const Vec3 &real_bearing_xyz, Vec3 &virtual_bearing_xyz) const {
    virtual_bearing_xyz = H_real_to_virtual_ * real_bearing_xyz;
}

Vec2 VirtualCamera::GetVirtualCameraFov() const {
    if (virtual_camera_model_ == nullptr) {
        return Vec2::Zero();
    }

    Vec3 horizontal_ray = Vec3::Zero();
    Vec3 vertical_ray = Vec3::Zero();
    virtual_camera_model_->LiftFromImagePlaneToBearingVector(Vec2(0, virtual_camera_model_->cy()), horizontal_ray);
    virtual_camera_model_->LiftFromImagePlaneToBearingVector(Vec2(virtual_camera_model_->cx(), 0), vertical_ray);
    return kRadToDeg * 2.0f *
           Vec2(std::atan2(horizontal_ray.head<2>().norm(), horizontal_ray.z()), std::atan2(vertical_ray.head<2>().norm(), vertical_ray.z()));
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
