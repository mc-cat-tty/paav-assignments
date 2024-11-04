#ifndef TRACKER_H_
#define TRACKER_H_

#include <tracker/Tracklet.h>
#include <limits>
#include <unordered_map>

struct Subject {
  Eigen::Vector2d coords;
  bool associated;

  inline double getX() const { return coords[0]; }
  inline double getY() const { return coords[1]; }
};

class Tracker {
public:
  Tracker();
  ~Tracker() = default;

  // handle tracklets
  void removeTracks();
  void addTracks();

  // associate tracklets and detections
  void dataAssociation();

  // track objects
  void track(const std::vector<double> &centroids_x,
             const std::vector<double> &centroids_y,
             bool lidarStatus);

  // getters
  const std::vector<Tracklet> &getTracks() const { return tracks_; }
  inline const double getLongestPath(int &track_id) const;

private:
  // tracklets
  std::vector<Tracklet> tracks_;
  int cur_id_;

  // association
  std::vector<std::pair<int, int>> associated_track_det_ids_;
  std::vector<Subject> subjects, subjects_old;

  // thresholds
  double distance_threshold_, distance_threshold_squared_;
  double covariance_threshold;
  int loss_threshold;
};

#endif // TRACKER_H_
