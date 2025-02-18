#include <tracker/Tracker.h>
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <limits>
#include <iomanip> 
#include <iostream> 

#define LOGGING_ON true
#include <logger.hpp>

Tracker::Tracker()
{
    cur_id_ = 0;
    
    distance_threshold_ = 5.0; // meters
    distance_threshold_squared_ = distance_threshold_*distance_threshold_;
    covariance_threshold = 1.5; 

    // number of frames the track has not been seen
    // Roughly loss_threshold/10 seconds withouth seeing the object have to pass to remove it
    loss_threshold = 200;

    longest_path = 0;
    longest_path_idx = 0;
  
}


/*
    This function removes tracks based on any strategy
*/
void Tracker::removeTracks() {
    // Monitor for how many frames the track is not linked to a subject
    std::erase_if(tracks_, [this](auto track){
        return track.getLossCount() > loss_threshold
            or track.getXCovariance() > covariance_threshold
            or track.getYCovariance() > covariance_threshold;
    });
}

/*
    This function add new tracks to the set of tracks ("tracks_" is the object that contains this)
*/
void Tracker::addTracks()
{
    // Adding not associated detections
    for (const auto &s : subjects) {
        if (not s.associated)
            tracks_.push_back(Tracklet(cur_id_++, s.getX(), s.getY()));
    }
}

void Tracker::updateLongestPath() {
    static auto logger = logger::Logger("LONGEST PATH");

    for (const auto &t : tracks_) {
        if (t.length > longest_path) {
            longest_path = t.length;
            longest_path_idx = t.getId();
        }
    }

    std::cout
        << "[Longest Path]"
        << longest_path_idx << ": "
        << std::setprecision(3) << longest_path << "m" << std::endl;
}


/*
    This function associates detections (centroids_x,centroids_y) with the tracks (tracks_)
    Input:
        associated_detection an empty vector to host the associated detection
        centroids_x & centroids_y measurements representing the detected objects
*/
void Tracker::dataAssociation()
{
    // This vector contains a pair of track and its corresponding subject
    auto prev_association_old = associated_track_det_ids_;
    associated_track_det_ids_.clear();
    static auto logger = logger::Logger("ASSOCIATION");

    int t_idx = -1;
    for (const auto &t : tracks_) {
        ++t_idx;
        int closest_point_id = -1;
        double min_dist = std::numeric_limits<double>::max();
        
        auto tracklet_coords = t.getCoords();

        unsigned s_idx = 0;
        for (const auto &s : subjects) {
            auto subject_coords = s.coords;
            auto delta = subject_coords - tracklet_coords;
            Eigen::Matrix2d covariance;
            covariance <<
                t.getXCovariance(), 0,
                0, t.getYCovariance();
            
            auto euclidean_squared = delta.squaredNorm();
            auto mahalanobis_squared = delta.transpose() * covariance.inverse() * delta;

            logger << t_idx << " - " << s_idx << ": "
                << euclidean_squared << " m (euclidean) vs "
                << mahalanobis_squared << "m (mahalanobis)";

            auto dist_squared = mahalanobis_squared;  // Choose either euclidean or mahalanobis
            
            if (dist_squared < min_dist) {
                min_dist = dist_squared;
                closest_point_id = s_idx;
            }

            ++s_idx;
        }

        // Associate the closest detection to a tracklet

        // Prediction gating
        if (min_dist > distance_threshold_squared_) continue;
    
        if (not subjects[closest_point_id].associated) {
            associated_track_det_ids_.emplace_back(closest_point_id, t_idx);
            subjects[closest_point_id].associated = true;
        }
        else {
            logger << "Contention just happened" << std::endl;
        }
    }
}

void Tracker::track(const std::vector<double> &centroids_x,
                    const std::vector<double> &centroids_y,
                    bool lidarStatus) {
    subjects_old.swap(subjects);
    subjects.clear();
    subjects.reserve(centroids_x.size());

    for (int i=0; i<centroids_x.size(); ++i) {
        subjects.emplace_back(
            Eigen::Vector2d(centroids_x[i], centroids_y[i]),
            false
        );
    }

    std::cout << "Initialized " << subjects.size() << " subjects" << std::endl;

    // For each track --> Predict the position of the tracklets
    for (auto &tracklet : tracks_) {
        tracklet.predict();
    }
    
    this->dataAssociation();

    // Update tracklets with the new detections
    for (auto &ass : associated_track_det_ids_)
    {
        auto det_id = ass.first;
        auto track_id = ass.second;
        tracks_[track_id].update(centroids_x[det_id], centroids_y[det_id], lidarStatus);
    }

    this->removeTracks();
    this->addTracks();

    this->updateLongestPath();
}
