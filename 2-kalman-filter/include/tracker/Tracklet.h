#ifndef TRACKLET_H_
#define TRACKLET_H_

#include <vector>
#include <cmath>

#include "KalmanFilter.h"

class Tracklet
{
public:
  Tracklet(int idTrack, double x, double y);
  ~Tracklet();

  void predict();
  void update(double x, double y, bool lidarStatus);

  // getters
  double getX() { return kf_.getX(); }
  double getY() { return kf_.getY(); }
  Eigen::Vector2d getCoords() { return Eigen::Vector2d(getX(), getY()); }

  double getXCovariance() { return kf_.getXCovariance(); }
  double getYCovariance() { return kf_.getYCovariance(); }
  int getId() { return id_; }

  inline void increaseLoss() { loss_count_++; }
  inline void zeroLoss() { loss_count_ = 0; }
  int getLossCount() { return loss_count_; }
  
  inline void setSubjectAssociated(bool a) { subject_associated = a; }
  inline bool isSubjectAssociated() const { return subject_associated; }

private:
  // filter
  KalmanFilter kf_;

  // tracklet id
  int id_;

  // number of loss since last update
  int loss_count_;

  bool subject_associated;
};

#endif // TRACKLET_H_
