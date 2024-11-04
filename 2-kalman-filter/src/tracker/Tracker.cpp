#include "tracker/Tracker.h"
#include "logger.hpp"
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <limits>

Tracker::Tracker()
{
    cur_id_ = 0;
    
    distance_threshold_ = 5.0; // meters
    distance_threshold_squared_ = distance_threshold_*distance_threshold_;
    covariance_threshold = 0.0; 

    // number of frames the track has not been seen
    // Roughly loss_threshold/10 seconds withouth seeing the object have to pass to remove it
    loss_threshold = 60;
}

Tracker::~Tracker() {}

/*
    This function removes tracks based on any strategy
*/
void Tracker::removeTracks() {
    // Monitor for how many frames the track is not linked to a subject
    for (auto &t : tracks_) {
        if (not t.isSubjectAssociated()) t.increaseLoss();
    }

    std::erase_if(tracks_, [this](auto track){ return track.getLossCount() > loss_threshold; });
}

/*
    This function add new tracks to the set of tracks ("tracks_" is the object that contains this)
*/
void Tracker::addTracks(const std::vector<double> &centroids_x, const std::vector<double> &centroids_y)
{
    // Adding not associated detections
    for (auto [subject_id, track_id] : subject_track_association) {
        if (!associated_detections[i])
            tracks_.push_back(Tracklet(cur_id_++, centroids_x[i], centroids_y[i]));
    }
    if 
}

/*
    This function associates detections (centroids_x,centroids_y) with the tracks (tracks_)
    Input:
        associated_detection an empty vector to host the associated detection
        centroids_x & centroids_y measurements representing the detected objects
*/
void Tracker::dataAssociation(const std::vector<double> &centroids_x, const std::vector<double> &centroids_y)
{

    // This vector contains a pair of track and its corresponding subject
    associated_track_det_ids_.clear();
    auto &logger = logger::Logger::getInstance();

    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        int closest_point_id = -1;
        double min_dist = std::numeric_limits<double>::max();
        
        auto tracklet_coords = tracks_[i].getCoords();

        for (size_t j = 0; j < associated_detections.size(); ++j) {
            auto subject_coords = Eigen::Vector2d(centroids_x[j], centroids_y[j]);
            auto delta = subject_coords - tracklet_coords;
            Eigen::Matrix2d covariance;
            covariance <<
                tracklet.getXCovariance(), 0,
                0, tracklet.getYCovariance();
            
            auto euclidean_squared = delta.squaredNorm();
            auto mahalanobis_squared = delta.transpose() * covariance.inverse() * delta;
            logger.logDistance(i, j, euclidean_squared, mahalanobis_squared);

            auto dist_squared = mahalanobis_squared;  // Choose either euclidean or mahalanobis
            
            if (dist_squared < min_dist) {
                min_dist = dist_squared;
                closest_point_id = j;
            }
        }

        // Associate the closest detection to a tracklet
        if (min_dist < distance_threshold_squared_ && !associated_detections[closest_point_id])
        {
            associated_track_det_ids_.push_back(std::make_pair(closest_point_id, i));
            associated_detections[closest_point_id] = true;
        }
    }
}

void Tracker::track(const std::vector<double> &centroids_x,
                    const std::vector<double> &centroids_y,
                    bool lidarStatus) {
    
    // For each track --> Predict the position of the tracklets
    for (auto &tracklet : tracks_) {
        tracklet.predict();
    }
    
    this->dataAssociation(centroids_x, centroids_y);

    // Update tracklets with the new detections
    for (int i = 0; i < associated_track_det_ids_.size(); ++i)
    {
        auto det_id = associated_track_det_ids_[i].first;
        auto track_id = associated_track_det_ids_[i].second;
        tracks_[track_id].update(centroids_x[det_id], centroids_y[det_id], lidarStatus);
    }

    this->removeTracks();
    this->addTracks(centroids_x, centroids_y);
}
