#ifndef _SENSOR_MODEL_VIRTUAL_CAMERA_H_
#define _SENSOR_MODEL_VIRTUAL_CAMERA_H_

#include "basic_type.h"
#include "camera_basic.h"
#include "memory"

namespace SENSOR_MODEL {

/* Class VirtualCamera Declaration. */
class VirtualCamera {

public:
    struct Options {
        const Quat kTargetQwc = Quat::Identity();
        const int32_t kVirtualImageRows = 100;
        const int32_t kVirtualImageCols = 100;
        const float kVirtualCameraFocusLength = 400.0f;
    };

public:
    VirtualCamera() = default;
    virtual ~VirtualCamera() = default;

    bool GenerateMaphex();
    bool RemapVirtualCameraImage();

    // Reference for member variables.
    Options &options() { return options_; }
    std::unique_ptr<CameraBasic> &raw_camera_model() { return raw_camera_model_; }
    MatImg &virtual_camera_image() { return virtual_camera_image_; }
    MatImg &virtual_camera_mask() { return virtual_camera_mask_; }

    // Const reference for member variables.
    const Options &options() const { return options_; }
    const std::unique_ptr<CameraBasic> &raw_camera_model() const { return raw_camera_model_; }
    const MatImg &virtual_camera_image() const { return virtual_camera_image_; }
    const MatImg &virtual_camera_mask() const { return virtual_camera_mask_; }

private:
    Options options_;
    std::unique_ptr<CameraBasic> raw_camera_model_ = nullptr;
    MatImg virtual_camera_image_;
    MatImg virtual_camera_mask_;
    Mat maphex_row_;
    Mat maphex_col_;
};

}

#endif // end of _SENSOR_MODEL_VIRTUAL_CAMERA_H_
