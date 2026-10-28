#include "lidar.h"

#include "slam_operations.h"

#include "algorithm"
#include "cmath"
#include "utility"

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

float Lidar::ComputeRatioOfZeroIntensityPoints(const LidarMeasurement &measure) {
    float ratio = 0.0;
    if (measure.intensity_of_points.empty()) {
        return ratio;
    }

    int32_t cnt = 0;
    for (const float &intensity: measure.intensity_of_points) {
        if (intensity == 0) {
            ++cnt;
        }
    }
    return static_cast<float>(cnt) / static_cast<float>(measure.intensity_of_points.size());
}

void Lidar::DownsampleLidarScan(const std::vector<Vec3> &all_raw_p_l, const uint32_t max_num_of_downsampled_points,
                              std::vector<Vec3> &all_downsampled_p_l, const float radius, const float voxel_step) {
    if (&all_raw_p_l == &all_downsampled_p_l) {
        std::vector<Vec3> sampled_points;
        DownsampleLidarScan(all_raw_p_l, max_num_of_downsampled_points, sampled_points, radius, voxel_step);
        all_downsampled_p_l.swap(sampled_points);
        return;
    }

    all_downsampled_p_l.clear();
    RETURN_IF(max_num_of_downsampled_points == 0 || all_raw_p_l.empty());
    RETURN_IF(!std::isfinite(radius) || radius <= 0.0f || !std::isfinite(voxel_step) || voxel_step <= 0.0f);
    const uint64_t invalid_index = static_cast<uint64_t>(-1);
    const uint64_t point_num = all_raw_p_l.size();
    all_downsampled_p_l.reserve(std::min<uint64_t>(max_num_of_downsampled_points, point_num));
    const auto is_valid = [radius](const Vec3 &point) {
        return point.allFinite() && point.cwiseAbs().maxCoeff() <= radius;
    };
    if (point_num <= max_num_of_downsampled_points) {
        for (const Vec3 &point: all_raw_p_l) {
            CONTINUE_IF(!is_valid(point));
            all_downsampled_p_l.emplace_back(point);
        }
        return;
    }

    // Build voxel buckets.
    struct Voxel {
        uint64_t first = 0;
        uint64_t count = 0;
        uint64_t quota = 0;
        uint64_t error = 0;
    };
    uint64_t capacity = 16;
    while (capacity / 2 < point_num) {
        capacity *= 2;
    }
    std::vector<std::pair<uint64_t, uint64_t>> table(capacity, {0, invalid_index});
    std::vector<Voxel> voxels;
    voxels.reserve(std::min<uint64_t>(max_num_of_downsampled_points, point_num));
    std::vector<uint64_t> point_voxels(point_num, invalid_index);
    const uint64_t mask = capacity - 1;
    const float inverse_step = 1.0f / voxel_step;
    const auto coordinate = [](const float value) {
        const int32_t truncated = static_cast<int32_t>(value);
        return static_cast<uint64_t>(truncated - (value < static_cast<float>(truncated)) + 1048576);
    };
    const auto hash = [](uint64_t key) {
        key ^= key >> 30;
        key *= 0xbf58476d1ce4e5b9ULL;
        key ^= key >> 27;
        key *= 0x94d049bb133111ebULL;
        return key ^ (key >> 31);
    };
    uint64_t valid_count = 0;
    for (uint64_t i = 0; i < point_num; ++i) {
        const Vec3 &point = all_raw_p_l[i];
        CONTINUE_IF(!is_valid(point));
        const Vec3 scaled = point * inverse_step;
        CONTINUE_IF(!scaled.allFinite() || scaled.minCoeff() < -1048576.0f || scaled.maxCoeff() >= 1048576.0f);
        // Pack three signed 21-bit voxel indices.
        const uint64_t key = (coordinate(scaled.x()) << 42) | (coordinate(scaled.y()) << 21) | coordinate(scaled.z());
        uint64_t slot = hash(key) & mask;
        while (table[slot].second != invalid_index && table[slot].first != key) {
            slot = (slot + 1) & mask;
        }
        auto &entry = table[slot];
        if (entry.second == invalid_index) {
            entry = {key, voxels.size()};
            voxels.emplace_back(Voxel {i, 0, 0, 0});
        }
        point_voxels[i] = entry.second;
        ++voxels[entry.second].count;
        ++valid_count;
    }

    const uint64_t budget = std::min<uint64_t>(max_num_of_downsampled_points, valid_count);
    RETURN_IF(budget == 0);
    if (voxels.size() >= budget) {
        // Select one point from each evenly spaced voxel.
        const uint64_t stride = voxels.size() / budget;
        const uint64_t remainder = voxels.size() % budget;
        uint64_t voxel = 0;
        uint64_t error = 0;
        for (uint64_t i = 0; i < budget; ++i) {
            all_downsampled_p_l.emplace_back(all_raw_p_l[voxels[voxel].first]);
            voxel += stride;
            error += remainder;
            if (error >= budget) {
                ++voxel;
                error -= budget;
            }
        }
        return;
    }

    // Distribute remaining quotas among occupied voxels.
    std::vector<uint64_t> active_voxels(voxels.size());
    for (uint64_t i = 0; i < active_voxels.size(); ++i) {
        active_voxels[i] = i;
    }
    uint64_t cursor = 0;
    for (uint64_t assigned = 0; assigned < budget; ++assigned) {
        auto &voxel = voxels[active_voxels[cursor]];
        if (++voxel.quota == voxel.count) {
            active_voxels[cursor] = active_voxels.back();
            active_voxels.pop_back();
        } else {
            ++cursor;
        }
        if (cursor == active_voxels.size()) {
            cursor = 0;
        }
    }

    // Select evenly spaced points within each voxel.
    for (auto &voxel: voxels) {
        voxel.error = voxel.count / 2;
    }
    for (uint64_t i = 0; i < point_num; ++i) {
        CONTINUE_IF(point_voxels[i] == invalid_index);
        auto &voxel = voxels[point_voxels[i]];
        voxel.error += voxel.quota;
        if (voxel.error >= voxel.count) {
            all_downsampled_p_l.emplace_back(all_raw_p_l[i]);
            voxel.error -= voxel.count;
        }
    }
}

void Lidar::ElevationCompensation(const std::vector<Vec3> &all_raw_p_l, const float elevation_rad, std::vector<Vec3> &all_p_l) {
    const float c = std::cos(elevation_rad);
    const float s = std::sin(elevation_rad);
    all_p_l.resize(all_raw_p_l.size());
    for (uint32_t i = 0; i < all_raw_p_l.size(); ++i) {
        // Copy the point so input and output may refer to the same vector.
        const Vec3 p = all_raw_p_l[i];
        const float r = std::hypot(p.x(), p.y());
        if (r < kZeroFloat) {
            // Azimuth is undefined near the Z axis.
            all_p_l[i] = p;
            continue;
        }
        const float horizontal = r * c - p.z() * s;
        const float z = p.z() * c + r * s;
        all_p_l[i] = Vec3(p.x() * (horizontal / r), p.y() * (horizontal / r), z);
    }
}

}  // namespace sensor_model
