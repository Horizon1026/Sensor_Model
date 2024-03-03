#include "gnss.h"
#include "math_kinematics.h"

namespace SENSOR_MODEL {

namespace {
    constexpr double kWgs84SemiMajorAxisInMeter = 6378137.0;
    constexpr double kWgs84SemiMinorAxisInMeter = 6356752.31414;
}

Vec3 Gnss::ConvertLlaToNed(const GnssMeasurement &origin_lla, const GnssMeasurement &lla) {
    TVec3<double> ned_position = TVec3<double>::Zero();

    const double delta_lon = lla.longitude_deg - origin_lla.longitude_deg;
    const double delta_lat = lla.latitude_deg - origin_lla.latitude_deg;
    const double radius = origin_lla.altitude_m + kWgs84SemiMajorAxisInMeter;

    ned_position.x() = radius * std::cos(lla.latitude_deg * kDegToRadDouble) * delta_lon;
    ned_position.y() = radius * delta_lat;
    ned_position.z() = lla.altitude_m - origin_lla.altitude_m;

    return ned_position.cast<float>();
}

}
