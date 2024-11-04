#ifndef TRACKER_H_
#define TRACKER_H_

#include "tracker/Tracklet.h"
#include <unordered_map>

class Tracker
{
public:
  Tracker();
  ~Tracker();

  // handle tracklets
  void removeTracks();
  void addTracks(const std::vector<double> &centroids_x,
                 const std::vector<double> &centroids_y);

  // associate tracklets and detections
  void dataAssociation(const std::vector<double> &centroids_x,
                       const std::vector<double> &centroids_y);

  // track objects
  void track(const std::vector<double> &centroids_x,
             const std::vector<double> &centroids_y,
             bool lidarStatus);

  // getters
  const std::vector<Tracklet> &getTracks() { return tracks_; }

private:
  // tracklets
  std::vector<Tracklet> tracks_;
  int cur_id_;

  // association
  std::unordered_map<int, int> subject_track_association;  ///< Key-val pairs organized as subject_idx-track_id
  std::unordered_map<int, unsigned> unassociated_num;  ///< Key-val pairs organized as subject_id-unassociation_counter

  // thresholds
  double distance_threshold_, distance_threshold_squared_;
  double covariance_threshold;
  int loss_threshold;
};

#endif // TRACKER_H_
