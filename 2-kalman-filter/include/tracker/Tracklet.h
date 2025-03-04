#ifndef TRACKLET_H_
#define TRACKLET_H_

#include <vector>
#include <cmath>

#include <tracker/KalmanFilter.h>

class Tracklet
{
public:
  Tracklet(int idTrack, double x, double y);
  ~Tracklet() = default;

  void predict();
  void update(double x, double y, bool lidarStatus);

  // getters
  double getX() const { return kf_.getX(); }
  double getY() const { return kf_.getY(); }
  Eigen::Vector2d getCoords() const { return Eigen::Vector2d(getX(), getY()); }

  double getXCovariance() const { return kf_.getXCovariance(); }
  double getYCovariance() const { return kf_.getYCovariance(); }
  int getId() const { return id_; }

  int getLossCount() { return loss_count_; }
  double length;

private:
  // filter
  KalmanFilter kf_;

  // tracklet id
  int id_;

  // number of loss since last update
  int loss_count_;

};

#endif // TRACKLET_H_
