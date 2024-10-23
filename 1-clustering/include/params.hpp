#pragma once
#include <string>
#include <exception>
#include <Eigen/Core>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace params {
    class Params {
        public:
            Eigen::Vector3f voxel_leaf_size, crop_box_min, crop_box_max;

            static Params& getInstance() {
                static Params instance;
                return instance;
            }

            void loadFromJson(std::string filename) {
                using namespace boost::property_tree;
                auto tree = ptree();
                read_json(filename, tree);

                auto extract_3d_vector = [&tree](std::string param_name) -> Eigen::Vector3f {
                    Eigen::Vector3f res;
                    unsigned i=0;
                    std::ranges::for_each(
                        tree.get_child(param_name),
                        [&i, &res](auto &val){
                            assert(val.first.empty());
                            res[i++] = std::stof(val.second.data());
                        }
                    );
                    return res;
                };

                
                voxel_leaf_size = extract_3d_vector("downsampling_leaf_size");
                crop_box_min = extract_3d_vector("cropping.min"); 
                crop_box_max = extract_3d_vector("cropping.max");
            }

        private:
            Params() = default;
            Params(const Params&) = delete;
            Params(Params&&) = delete;
            Params& operator=(const Params&) = delete;
    };
}