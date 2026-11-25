#include "gnss.h"

#include "slam_operations.h"

#include "slam_basic_math.h"
#include "slam_log_reporter.h"

#include "cmath"
#include "cstdio"
#include "fstream"

namespace sensor_model {

bool Gnss::ParseMeasuresFromFile(const std::string &file_path, const double &time_stamp_offset_s, std::vector<GnssMeasurement> &measures) {
    std::ifstream file(file_path);
    std::string line;
    RETURN_FALSE_IF(!file.is_open() || !std::getline(file, line));

    measures.clear();
    while (std::getline(file, line)) {
        double data[8] = {};
        CONTINUE_IF(
            std::sscanf(line.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6], &data[7]) != 8);
        GnssMeasurement measure;
        measure.time_stamp_s = data[0] * 1e-9 - time_stamp_offset_s;
        const int32_t fixed_type = static_cast<int32_t>(std::round(data[1]));
        const bool is_valid = fixed_type > static_cast<int32_t>(GnssFixedType::kInvalid) && fixed_type <= static_cast<int32_t>(GnssFixedType::kSimulatorMode);
        measure.fixed_type = is_valid ? static_cast<GnssFixedType>(fixed_type) : GnssFixedType::kInvalid;
        measure.is_lla_valid = is_valid;
        measure.longitude_deg = data[5];
        measure.latitude_deg = data[6];
        measure.altitude_m = data[7];
        measure.is_vel_valid = is_valid;
        measure.vel_east_mps = static_cast<float>(data[2]);
        measure.vel_north_mps = static_cast<float>(data[3]);
        measure.vel_up_mps = static_cast<float>(data[4]);
        measures.emplace_back(measure);
    }
    return !measures.empty();
}

namespace {
    constexpr double kWgs84SemiMajorAxisInMeter = 6378137.0;
    constexpr double kWgs84SemiMinorAxisInMeter = 6356752.31414;
}  // namespace

TVec3<double> Gnss::ConvertLlaToEnu(const GnssMeasurement &origin_lla, const GnssMeasurement &lla) {
    TVec3<double> enu_position = TVec3<double>::Zero();

    const double delta_lon = FormatDegree(lla.longitude_deg - origin_lla.longitude_deg);
    const double delta_lat = FormatDegree(lla.latitude_deg - origin_lla.latitude_deg);
    const double radius = origin_lla.altitude_m + kWgs84SemiMajorAxisInMeter;

    enu_position.x() = radius * std::cos(lla.latitude_deg * kDegToRadDouble) * delta_lon * kDegToRadDouble;
    enu_position.y() = radius * delta_lat * kDegToRadDouble;
    enu_position.z() = lla.altitude_m - origin_lla.altitude_m;

    return enu_position;
}

GnssMeasurement Gnss::ConvertEnuToLla(const GnssMeasurement &origin_lla, const TVec3<double> &enu) {
    GnssMeasurement lla;
    const double radius_inv = 1.0 / (origin_lla.altitude_m + kWgs84SemiMajorAxisInMeter);

    const double delta_lat = enu.y() * radius_inv * kRadToDegDouble;
    lla.latitude_deg = FormatDegree(origin_lla.latitude_deg + delta_lat);

    const double delta_lon = enu.x() * radius_inv / std::cos(lla.latitude_deg * kDegToRadDouble) * kRadToDegDouble;
    lla.longitude_deg = FormatDegree(origin_lla.longitude_deg + delta_lon);

    lla.altitude_m = enu.z() + origin_lla.altitude_m;

    return lla;
}

double Gnss::FormatDegree(const double abnormal_degree) {
    if (abnormal_degree < -180.0) {
        return abnormal_degree + 180.0 * 2.0;
    } else if (abnormal_degree > 180.0) {
        return abnormal_degree - 180.0 * 2.0;
    }

    return abnormal_degree;
}

}  // namespace sensor_model
