#pragma once

#include <pcl/types.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <Eigen/Core>
#include <iterator>
#include <ranges>
#include <limits>

namespace utils {
    using Pxyz = pcl::PointXYZ;
    using PcPtr = pcl::PointCloud<Pxyz>::Ptr;

    float distance_from_origin(std::ranges::input_range auto&& cluster) {
        const auto origin = Pxyz(0, 0, 0);
        float min_dist = std::numeric_limits<float>::max();
        bool dist_processed = false;
        
        for (const auto &p : cluster) {
            auto new_dist = pcl::euclideanDistance(origin, p);
            if (new_dist < min_dist) {
                min_dist = new_dist;
                dist_processed = true;
            }
        }

        return dist_processed ? min_dist : -1;
        // Negative distance logically mark that no distance has been processed
    }

    float front_distace_from_origin(PcPtr cluster) {
        auto in_front = [](Pxyz p) { return p.x > 0; };
        return distance_from_origin(*cluster | std::views::filter(in_front));
    }

    // Here the threshold is slightly higher than the one proposed by the assignemnt text
    // to better emphasize the behaviour of this function
    float is_dangerous_obstacle(PcPtr cluster, float distance_threshold = 10.0f) {
        bool unsafe_distance = distance_from_origin(*cluster) < distance_threshold;
        bool unsafe_position = front_distace_from_origin(cluster) > 0;
        return unsafe_distance && unsafe_position;
    }
}