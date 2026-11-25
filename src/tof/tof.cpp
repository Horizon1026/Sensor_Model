#include "tof.h"

#include "slam_operations.h"

#include "algorithm"
#include "cmath"
#include "cstdio"
#include "fstream"

namespace sensor_model {

bool Tof::ParseMeasuresFromFile(const std::string &file_path, const double &time_stamp_offset_s, std::vector<TofMeasurement> &measures) {
    std::ifstream file(file_path);
    std::string line;
    RETURN_FALSE_IF(!file.is_open() || !std::getline(file, line));

    measures.clear();
    while (std::getline(file, line)) {
        double data[3] = {};
        CONTINUE_IF(std::sscanf(line.c_str(), "%lf,%lf,%lf", &data[0], &data[1], &data[2]) != 3);
        TofMeasurement measure;
        measure.time_stamp_s = data[0] * 1e-9 - time_stamp_offset_s;
        CONTINUE_IF(!std::isfinite(data[0]));
        measure.depth_m = static_cast<float>(data[2]);
        measure.is_depth_valid = data[1] == 1 && std::isfinite(measure.depth_m) && measure.depth_m > 0.0f;
        measures.emplace_back(measure);
    }
    std::stable_sort(measures.begin(), measures.end(), [](const auto &lhs, const auto &rhs) {
        return lhs.time_stamp_s < rhs.time_stamp_s;
    });
    measures.erase(std::unique(measures.begin(), measures.end(),
                               [](const auto &lhs, const auto &rhs) {
                                   return lhs.time_stamp_s == rhs.time_stamp_s;
                               }),
                   measures.end());
    return !measures.empty();
}

}  // namespace sensor_model
