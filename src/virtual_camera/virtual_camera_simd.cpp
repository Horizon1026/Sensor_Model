#include "virtual_camera.h"

#if defined(SENSOR_MODEL_USE_HIGHWAY)

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "virtual_camera_simd.cpp"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"

HWY_BEFORE_NAMESPACE();
namespace sensor_model {
namespace HWY_NAMESPACE {

    namespace hn = hwy::HWY_NAMESPACE;

    void GenerateMaphexHighway(const float *rotation, const float *bearing_x, const float *bearing_y, const float *bearing_z, const uint8_t *valid,
                               const uint64_t count, const BasicCameraModel<float> *camera, float *map_col, float *map_row) {
        const hn::CappedTag<float, 16> d {};
        const uint32_t lanes = static_cast<uint32_t>(hn::Lanes(d));
        const auto r00 = hn::Set(d, rotation[0]);
        const auto r01 = hn::Set(d, rotation[3]);
        const auto r02 = hn::Set(d, rotation[6]);
        const auto r10 = hn::Set(d, rotation[1]);
        const auto r11 = hn::Set(d, rotation[4]);
        const auto r12 = hn::Set(d, rotation[7]);
        const auto r20 = hn::Set(d, rotation[2]);
        const auto r21 = hn::Set(d, rotation[5]);
        const auto r22 = hn::Set(d, rotation[8]);
        std::array<float, 16> rotated_x {};
        std::array<float, 16> rotated_y {};
        std::array<float, 16> rotated_z {};

        uint64_t index = 0;
        for (; index + lanes <= count; index += lanes) {
            const auto x = hn::LoadU(d, bearing_x + index);
            const auto y = hn::LoadU(d, bearing_y + index);
            const auto z = hn::LoadU(d, bearing_z + index);
            hn::StoreU(hn::Add(hn::Add(hn::Mul(r00, x), hn::Mul(r01, y)), hn::Mul(r02, z)), d, rotated_x.data());
            hn::StoreU(hn::Add(hn::Add(hn::Mul(r10, x), hn::Mul(r11, y)), hn::Mul(r12, z)), d, rotated_y.data());
            hn::StoreU(hn::Add(hn::Add(hn::Mul(r20, x), hn::Mul(r21, y)), hn::Mul(r22, z)), d, rotated_z.data());
            for (uint32_t lane = 0; lane < lanes; ++lane) {
                map_col[index + lane] = -1;
                map_row[index + lane] = -1;
                if (!valid[index + lane]) {
                    continue;
                }
                const Vec3 bearing_xyz(rotated_x[lane], rotated_y[lane], rotated_z[lane]);
                Vec2 pixel_uv = Vec2::Zero();
                if (camera->LiftFromBearingVectorToRawImagePlane(bearing_xyz, pixel_uv)) {
                    map_col[index + lane] = pixel_uv.x();
                    map_row[index + lane] = pixel_uv.y();
                }
            }
        }

        // Handle pixels that do not fill an entire SIMD batch.
        const Mat3 rotation_matrix = Eigen::Map<const Mat3>(rotation);
        for (; index < count; ++index) {
            map_col[index] = -1;
            map_row[index] = -1;
            if (!valid[index]) {
                continue;
            }
            const Vec3 virtual_bearing_xyz(bearing_x[index], bearing_y[index], bearing_z[index]);
            const Vec3 real_bearing_xyz = rotation_matrix * virtual_bearing_xyz;
            Vec2 pixel_uv = Vec2::Zero();
            if (camera->LiftFromBearingVectorToRawImagePlane(real_bearing_xyz, pixel_uv)) {
                map_col[index] = pixel_uv.x();
                map_row[index] = pixel_uv.y();
            }
        }
    }

}  // namespace HWY_NAMESPACE
}  // namespace sensor_model
HWY_AFTER_NAMESPACE();

#if HWY_ONCE
namespace sensor_model {

HWY_EXPORT(GenerateMaphexHighway);

void VirtualCamera::GenerateMaphexSimd() {
    HWY_DYNAMIC_DISPATCH(GenerateMaphexHighway)
    (H_virtual_to_real_.data(), bearing_map_xyz_[0].data(), bearing_map_xyz_[1].data(), bearing_map_xyz_[2].data(), bearing_map_valid_.data(),
     static_cast<uint64_t>(maphex_col_.size()), real_camera_model_.get(), maphex_col_.data(), maphex_row_.data());
}

}  // namespace sensor_model
#endif

#else

namespace sensor_model {

void VirtualCamera::GenerateMaphexSimd() { GenerateMaphexCpu(); }

}  // namespace sensor_model

#endif
