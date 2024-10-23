#pragma once

#include <pcl/types.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
 #include <Eigen/Core>  

namespace utils {
    using Pxyz = pcl::PointXYZ;
    using PcPtr = pcl::PointCloud<Pxyz>::Ptr;

    float distance_from_origin(PcPtr cluster) {
        const auto origin = Pxyz(0, 0, 0);
        float min_dist = pcl::euclideanDistance(
            origin,
            (*cluster)[0]
        );

        for (const auto &p : *cluster) {
            auto new_dist = pcl::euclideanDistance(origin, p);
            if (new_dist < min_dist) min_dist = new_dist;
        }

        return min_dist;
    }
}