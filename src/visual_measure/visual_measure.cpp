#include "visual_measure.h"

#include "slam_operations.h"

#include "fstream"
#include "sstream"
#include "utility"

namespace sensor_model {

bool VisualMeasure::ParseOneMeasureFromFile(const std::string &file_path, VisualMixMeasure &measure) {
    std::ifstream file(file_path);
    std::string line;
    RETURN_FALSE_IF(!file.is_open() || !std::getline(file, line));

    measure.points_id.clear();
    measure.points_observes_per_frame.clear();
    while (std::getline(file, line)) {
        std::istringstream stream(line);
        std::vector<std::string> fields;
        std::string field;
        while (std::getline(stream, field, ',')) {
            fields.emplace_back(field);
        }
        if (!line.empty() && line.back() == ',') {
            fields.emplace_back();
        }
        CONTINUE_IF(fields.size() < 13 || fields[5].empty());

        PointsObservePerFrame observations;
        for (size_t i = 5; i + 7 < fields.size() && !fields[i].empty(); i += 8) {
            observations.emplace_back(PointsObservePerView {
                .raw_pixel_uv = Vec2(std::stof(fields[i]), std::stof(fields[i + 1])),
                .rectified_norm_xy = Vec2(std::stof(fields[i + 2]), std::stof(fields[i + 3])),
                .bearing_xyz = Vec3(std::stof(fields[i + 4]), std::stof(fields[i + 5]), std::stof(fields[i + 6])),
            });
        }
        measure.points_id.emplace_back(std::stoul(fields[1]));
        measure.points_observes_per_frame.emplace_back(std::move(observations));
    }
    return !measure.points_id.empty();
}

}  // namespace sensor_model
