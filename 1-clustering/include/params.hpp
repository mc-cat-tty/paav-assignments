#pragma once
#include <string>
#include <exception>
#include <Eigen/Core>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace params {
    class Params {
        public:
            Eigen::Vector4f
                voxel_leaf_size,
                crop_box_min, crop_box_max,
                ego_box_min, ego_box_max;
                
            bool render_raw_pc, render_filtered_pc, render_ground, render_ego;
            
            float
                ransac_max_iterations, inlier_admission_threshold,
                cluster_tolerance, cluster_min_threshold, cluster_max_threshold;

            static Params& getInstance() {
                static Params instance;
                return instance;
            }

            void loadFromJson(std::string filename) {
                using namespace boost::property_tree;
                auto tree = ptree();
                read_json(filename, tree);

                auto extract_3d_vector = [&tree](std::string param_name) -> Eigen::Vector4f {
                    Eigen::Vector4f res;
                    unsigned i=0;
                    std::ranges::for_each(
                        tree.get_child(param_name),
                        [&i, &res](auto &val){
                            assert(val.first.empty());
                            res[i++] = std::stof(val.second.data());
                        }
                    );
                    res[3] = 1;
                    return res;
                };

                
                voxel_leaf_size = extract_3d_vector("downsampling.voxel_leaf_size");
                crop_box_min = extract_3d_vector("cropping.min"); 
                crop_box_max = extract_3d_vector("cropping.max");
                ego_box_min = extract_3d_vector("ego_boundaries.min");
                ego_box_max = extract_3d_vector("ego_boundaries.max");

                render_raw_pc = tree.get<bool>("pointcloud_viewer.raw.render");
                render_filtered_pc = tree.get<bool>("pointcloud_viewer.filtered.render");
                render_ground = tree.get<bool>("pointcloud_viewer.ground_plane.render");
                render_ego = tree.get<bool>("pointcloud_viewer.ego.render");
                
                ransac_max_iterations = tree.get<float>("segmentation.ransac_max_iterations");
                inlier_admission_threshold = tree.get<float>("segmentation.inlier_admission_threshold");
                cluster_tolerance = tree.get<float>("clustering.tolerance");
                cluster_min_threshold = tree.get<float>("clustering.min_cluster_threshold");
                cluster_max_threshold = tree.get<float>("clustering.max_cluster_threshold");
            }

        private:
            Params() = default;
            Params(const Params&) = delete;
            Params(Params&&) = delete;
            Params& operator=(const Params&) = delete;
    };
}