#include "gnss.h"
#include "math_kinematics.h"

namespace SENSOR_MODEL {

namespace {
    constexpr double kWgs84SemiMajorAxisInMeter = 6378137.0;
    constexpr double kWgs84SemiMinorAxisInMeter = 6356752.31414;
}

TVec3<double> Gnss::ConvertLlaToNed(const GnssMeasurement &origin_lla, const GnssMeasurement &lla) {
    TVec3<double> ned_position = TVec3<double>::Zero();

    const double delta_lon = FormatDegree(lla.longitude_deg - origin_lla.longitude_deg);
    const double delta_lat = FormatDegree(lla.latitude_deg - origin_lla.latitude_deg);
    const double radius = origin_lla.altitude_m + kWgs84SemiMajorAxisInMeter;

    ned_position.x() = radius * std::cos(lla.latitude_deg * kDegToRadDouble) * delta_lon;
    ned_position.y() = radius * delta_lat;
    ned_position.z() = lla.altitude_m - origin_lla.altitude_m;

    return ned_position;
}

GnssMeasurement Gnss::ConvertNedToLla(const GnssMeasurement &origin_lla, const TVec3<double> &ned) {
    GnssMeasurement lla;
    const double radius = origin_lla.altitude_m + kWgs84SemiMajorAxisInMeter;

    const double delta_lat = ned.y() / radius;
    lla.latitude_deg = FormatDegree(origin_lla.latitude_deg + delta_lat);

    const double delta_lon = ned.x() / radius / std::cos(lla.latitude_deg * kDegToRadDouble);
    lla.longitude_deg = FormatDegree(origin_lla.longitude_deg + delta_lon);

    lla.altitude_m = ned.z() + origin_lla.altitude_m;

    return lla;
}

double Gnss::FormatDegree(const double abnormal_degree) {
    if (abnormal_degree < kPaiDouble * 0.5) {
        return abnormal_degree + kPaiDouble;
    }

    if (abnormal_degree > kPaiDouble * 0.5) {
        return abnormal_degree - kPaiDouble;
    }

    return abnormal_degree;
}

}
