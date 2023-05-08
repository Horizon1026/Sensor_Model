#include "imu_preintegrate.h"

namespace SENSOR_MODEL {

// Correct integrate block with delta bias_a and bias_g.
void ImuPreintegrateBlock::Correct(const Vec3 &delta_ba, const Vec3 &delta_bg) {
    p_ij_ += dp_dba() * delta_ba + dp_dbg() * delta_bg;
    v_ij_ += dv_dba() * delta_ba + dv_dbg() * delta_bg;

    const Vec3 temp = dr_dbg() * delta_bg;
    q_ij_ = q_ij_ * Utility::Exponent(temp).normalized();
    q_ij_.normalize();
}

}
