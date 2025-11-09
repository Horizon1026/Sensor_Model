#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "visualizor_3d.h"

#include <fstream>
#include <iostream>

#include "gnss.h"

using namespace SLAM_UTILITY;
using namespace SENSOR_MODEL;
using namespace SLAM_VISUALIZOR;

void LoadGnssMeasurements(const std::string &file_name, std::vector<TVec3<double>> &positions) {
    std::ifstream file(file_name.c_str());
    if (!file.is_open()) {
        ReportError("Failed to load data file " << file_name);
        return;
    }

    // Read first line. Use first lla to be origin.
    std::string one_line;
    std::getline(file, one_line);
    std::istringstream data_in_first_line(one_line);

    std::string type;
    GnssMeasurement gnss_origin;
    data_in_first_line >> type >> gnss_origin.time_stamp_s >> gnss_origin.latitude_deg >> gnss_origin.longitude_deg >> gnss_origin.altitude_m >>
        gnss_origin.yaw_ned_deg >> gnss_origin.is_lla_valid;

    Gnss gnss_model;
    const TVec3<double> first_position = gnss_model.ConvertLlaToEnu(gnss_origin, gnss_origin);
    positions.emplace_back(first_position);

    // Publish each line of data file.
    double average_residual = 0;
    while (std::getline(file, one_line) && !one_line.empty()) {
        std::istringstream data(one_line);

        GnssMeasurement gnss;
        data >> type >> gnss.time_stamp_s >> gnss.latitude_deg >> gnss.longitude_deg >> gnss.altitude_m >> gnss.yaw_ned_deg >> gnss.is_yaw_valid;

        const TVec3<double> enu_pos = gnss_model.ConvertLlaToEnu(gnss_origin, gnss);
        positions.emplace_back(enu_pos);

        // Double check transformation.
        GnssMeasurement gnss_cal = gnss_model.ConvertEnuToLla(gnss_origin, enu_pos);
        const double residual = std::sqrt((gnss.longitude_deg - gnss_cal.longitude_deg) * (gnss.longitude_deg - gnss_cal.longitude_deg) +
                                          (gnss.latitude_deg - gnss_cal.latitude_deg) * (gnss.latitude_deg - gnss_cal.latitude_deg) +
                                          (gnss.altitude_m - gnss_cal.altitude_m) * (gnss.altitude_m - gnss_cal.altitude_m));
        average_residual += residual;
    }
    average_residual /= static_cast<double>(positions.size());
    ReportInfo("Transform residual between LLA and ENU is " << average_residual);

    file.close();
}

int main(int argc, char **argv) {
    std::string gnss_file = "../examples/gnss_lla.txt";
    if (argc == 2) {
        gnss_file = argv[1];
    }

    ReportInfo(YELLOW ">> Test gnss model." RESET_COLOR);

    std::vector<TVec3<double>> positions;
    LoadGnssMeasurements(gnss_file, positions);

    Visualizor3D::camera_view().p_wc = Vec3(-50, 120, -300);
    Visualizor3D::Clear();
    for (const auto &position: positions) {
        Visualizor3D::points().emplace_back(PointType {
            .p_w = position.cast<float>(),
            .color = RgbColor::kCyan,
            .radius = 2,
        });
    }
    while (!Visualizor3D::ShouldQuit()) {
        Visualizor3D::Refresh("GNSS Trajectory", 30);
    }

    return 0;
}
