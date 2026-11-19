#ifndef _SENSOR_MODEL_VIRTUAL_CAMERA_H_
#define _SENSOR_MODEL_VIRTUAL_CAMERA_H_

#include "camera_model.h"

#include "basic_type.h"

#include "array"
#include "memory"
#include "string"
#include "vector"

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
    // Call after changing custom camera state not exposed by its intrinsic or distortion parameters.
    void InvalidateBearingMap() { bearing_map_camera_ptr_ = nullptr; }
    bool RemapVirtualCameraImage(const GrayImage &raw_image);
    bool RemapPixelUvFromVirtualCameraToRealCamera(const Vec2 &virtual_pixel_uv, Vec2 &real_distort_pixel_uv);
    bool RemapPixelUvFromRealCameraToVirtualCamera(const Vec2 &real_distort_pixel_uv, Vec2 &virtual_pixel_uv);
    // Rotate physical rays without projecting onto a normalized plane; ray length is preserved.
    void RemapBearingVectorFromVirtualCameraToRealCamera(const Vec3 &virtual_bearing_xyz, Vec3 &real_bearing_xyz) const;
    void RemapBearingVectorFromRealCameraToVirtualCamera(const Vec3 &real_bearing_xyz, Vec3 &virtual_bearing_xyz) const;

    Vec2 GetVirtualCameraFov() const;
    int32_t GetVirtualCameraImageRows() const;
    int32_t GetVirtualCameraImageCols() const;

    // Reference for member variables.
    Options &options() { return options_; }
    std::unique_ptr<BasicCameraModel<float>> &virtual_camera_model() { return virtual_camera_model_; }
    std::unique_ptr<BasicCameraModel<float>> &real_camera_model() { return real_camera_model_; }
    MatImg &virtual_camera_image() { return virtual_camera_image_; }
    MatImg &virtual_camera_mask() { return virtual_camera_mask_; }
    // Const reference for member variables.
    const Options &options() const { return options_; }
    const std::unique_ptr<BasicCameraModel<float>> &virtual_camera_model() const { return virtual_camera_model_; }
    const std::unique_ptr<BasicCameraModel<float>> &real_camera_model() const { return real_camera_model_; }
    const MatImg &virtual_camera_image() const { return virtual_camera_image_; }
    const MatImg &virtual_camera_mask() const { return virtual_camera_mask_; }

private:
    void GenerateBearingMap();
    void GenerateMaphexCpu();
    void GenerateMaphexSimd();

private:
    Options options_;
    std::unique_ptr<BasicCameraModel<float>> virtual_camera_model_ = nullptr;
    std::unique_ptr<BasicCameraModel<float>> real_camera_model_ = nullptr;
    MatImg virtual_camera_image_;
    MatImg virtual_camera_mask_;
    Mat maphex_row_;
    Mat maphex_col_;
    std::array<Mat, 3> bearing_map_xyz_ {};
    TMat<uint8_t> bearing_map_valid_ {};
    const BasicCameraModel<float> *bearing_map_camera_ptr_ = nullptr;
    std::string bearing_map_camera_name_ = "";
    std::array<int64_t, 4> bearing_map_integer_parameters_ {};
    std::vector<float> bearing_map_parameters_ {};
    Mat3 H_virtual_to_real_ = Mat3::Identity();
    Mat3 H_real_to_virtual_ = Mat3::Identity();
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_VIRTUAL_CAMERA_H_
