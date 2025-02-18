
#ifndef MAP_H_
#define MAP_H_

#include <algorithm>
#include <utility>

class Map {
public:
	
	struct single_landmark_s{

		int id_i ; // Landmark ID
		float x_f; // Landmark x-position in the map (global coordinates)
		float y_f; // Landmark y-position in the map (global coordinates)
	};

	std::vector<single_landmark_s> landmark_list ; // List of landmarks in the map

	std::pair<float, float> get_x_boundaries() const {
		auto cmp_x = [](const single_landmark_s& a, const single_landmark_s& b) { return a.x_f < b.x_f; };

		auto min_x = std::min_element(landmark_list.begin(), landmark_list.end(), cmp_x)->x_f;
		auto max_x = std::max_element(landmark_list.begin(), landmark_list.end(), cmp_x)->x_f;

		return std::make_pair(min_x, max_x);
	}

	std::pair<float, float> get_y_boundaries() const {
		auto cmp_y = [](const single_landmark_s& a, const single_landmark_s& b) { return a.y_f < b.y_f; };

		auto min_y = std::min_element(landmark_list.begin(), landmark_list.end(), cmp_y)->y_f;
		auto max_y = std::max_element(landmark_list.begin(), landmark_list.end(), cmp_y)->y_f;

		return std::make_pair(min_y, max_y);
	}
};



#endif /* MAP_H_ */
