#ifndef _SENSOR_MODEL_VIRTUAL_CAMERA_H_
#define _SENSOR_MODEL_VIRTUAL_CAMERA_H_

#include "basic_type.h"
#include "camera_model.h"
#include "memory"

namespace sensor_model {

/* Class VirtualCamera Declaration. */
class VirtualCamera {

public:
    struct Options {
        Quat kQrv = Quat::Identity();
        int32_t kDefaultVirtualCameraImageRows = 640;
        int32_t kDefaultVirtualCameraImageCols = 640;
        float kDefaultVirtualCameraFocalLength = 450.0f;
    };

public:
    VirtualCamera() = default;
    virtual ~VirtualCamera() = default;

    bool CreateDefaultVirtualCameraModel();
    bool GenerateMaphex();
    bool GenerateMaphex(const Quat &q_rv);
    bool RemapVirtualCameraImage(const GrayImage &raw_image);
    bool RemapPixelUvFromVirtualCameraToRawCamera(const Vec2 &virtual_pixel_uv, Vec2 &raw_distort_pixel_uv);
    bool RemapPixelUvFromRawCameraToVirtualCamera(const Vec2 &raw_distort_pixel_uv, Vec2 &virtual_pixel_uv);

    Vec2 GetVirtualCameraFov() const;
    int32_t GetVirtualCameraImageRows() const;
    int32_t GetVirtualCameraImageCols() const;

    // Reference for member variables.
    Options &options() { return options_; }
    std::unique_ptr<CameraPinhole> &virtual_camera_model() { return virtual_camera_model_; }
    std::unique_ptr<CameraPinhole> &real_camera_model() { return real_camera_model_; }
    MatImg &virtual_camera_image() { return virtual_camera_image_; }
    MatImg &virtual_camera_mask() { return virtual_camera_mask_; }
    // Const reference for member variables.
    const Options &options() const { return options_; }
    const std::unique_ptr<CameraPinhole> &virtual_camera_model() const { return virtual_camera_model_; }
    const std::unique_ptr<CameraPinhole> &real_camera_model() const { return real_camera_model_; }
    const MatImg &virtual_camera_image() const { return virtual_camera_image_; }
    const MatImg &virtual_camera_mask() const { return virtual_camera_mask_; }

private:
    Options options_;
    std::unique_ptr<CameraPinhole> virtual_camera_model_ = nullptr;
    std::unique_ptr<CameraPinhole> real_camera_model_ = nullptr;
    MatImg virtual_camera_image_;
    MatImg virtual_camera_mask_;
    Mat maphex_row_;
    Mat maphex_col_;
    Mat3 H_cc0_ = Mat3::Identity();
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_VIRTUAL_CAMERA_H_
