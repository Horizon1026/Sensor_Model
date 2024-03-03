#include "datatype_basic.h"
#include "slam_operations.h"
#include "log_report.h"

#include <fstream>
#include <iostream>

#include "gnss.h"

using namespace SLAM_UTILITY;
using namespace SENSOR_MODEL;

void LoadSimDataAndPublish(const std::string &file_name, std::vector<Vec3> &positions) {
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
    data_in_first_line >> type >> gnss_origin.time_stamp_s >> gnss_origin.longitude_deg >>
        gnss_origin.latitude_deg >> gnss_origin.altitude_m >> gnss_origin.yaw_north_rad >>
        gnss_origin.is_fixed;

    Gnss gnss_model;
    const Vec3 first_position = gnss_model.ConvertLlaToNed(gnss_origin, gnss_origin);
    positions.emplace_back(first_position);

    // Publish each line of data file.
    while (std::getline(file, one_line) && !one_line.empty()) {
        std::istringstream data(one_line);

        GnssMeasurement gnss;
        data >> type >> gnss.time_stamp_s >> gnss.longitude_deg >> gnss.latitude_deg >>
            gnss.altitude_m >> gnss.yaw_north_rad >> gnss.is_fixed;

        const Vec3 position = gnss_model.ConvertLlaToNed(gnss_origin, gnss);
        positions.emplace_back(position);
    }

    file.close();
}

int main(int argc, char **argv) {
    std::string gnss_file = "../examples/gnss_lla.txt";
    if (argc == 2) {
        gnss_file = argv[1];
    }

    ReportInfo(YELLOW ">> Test gnss model." RESET_COLOR);
    LogFixPercision(5);

    std::vector<Vec3> positions;
    LoadSimDataAndPublish(gnss_file, positions);

    for (const auto &position : positions) {
        ReportInfo(LogVec(position));
    }

    return 0;
}
