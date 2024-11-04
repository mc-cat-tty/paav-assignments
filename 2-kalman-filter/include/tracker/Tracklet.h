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

  int getLossCount() { return loss_count_; }

private:
  // filter
  KalmanFilter kf_;

  // tracklet id
  int id_;

  // number of loss since last update
  int loss_count_;
};

#endif // TRACKLET_H_
