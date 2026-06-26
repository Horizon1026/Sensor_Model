#include "lidar.h"
#include "slam_operations.h"

namespace sensor_model {

void Lidar::RemoveLowIntensityPoints(LidarMeasurement &measure) {
    const uint32_t point_num = measure.intensity_of_points.size();
    if (point_num == 0) {
        return;
    }
    // Build status vector. 1 = keep, 0 = remove.
    std::vector<uint8_t> status(point_num, 1);

    // Remove all points with intensity == 0.
    uint32_t remaining_count = 0;
    for (uint32_t i = 0; i < point_num; ++i) {
        if (measure.intensity_of_points[i] == 0.0f) {
            status[i] = 0;
        } else {
            ++remaining_count;
        }
    }

    // Among remaining points, remove the lowest 20% by intensity.
    if (remaining_count > 0) {
        // Collect indices of points that survived step 1.
        std::vector<int32_t> sorted_indices;
        sorted_indices.reserve(remaining_count);
        for (uint32_t i = 0; i < point_num; ++i) {
            if (status[i] == 1) {
                sorted_indices.push_back(static_cast<int32_t>(i));
            }
        }

        // Sort indices by intensity (ascending).
        slam_utility::SlamOperation::ArgSort(measure.intensity_of_points.data(), point_num, sorted_indices);

        // Mark the lowest (defaultly 20%) as removed.
        const uint32_t count_to_remove = static_cast<uint32_t>(options_.kRatioOfLowIntensityPointsToBeRemoved * remaining_count);
        for (uint32_t j = 0; j < count_to_remove; ++j) {
            status[sorted_indices[j]] = 0;
        }
    }

    // Apply ReduceVectorByStatus to all non-empty vectors.
    if (!measure.time_stamp_s_of_points.empty()) {
        slam_utility::SlamOperation::ReduceVectorByStatus(status, measure.time_stamp_s_of_points);
    }
    if (!measure.raw_points.empty()) {
        slam_utility::SlamOperation::ReduceVectorByStatus(status, measure.raw_points);
    }
    if (!measure.intensity_of_points.empty()) {
        slam_utility::SlamOperation::ReduceVectorByStatus(status, measure.intensity_of_points);
    }
    if (!measure.undistorted_points.empty()) {
        slam_utility::SlamOperation::ReduceVectorByStatus(status, measure.undistorted_points);
    }
}

}  // namespace sensor_model
