#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <eigen3/Eigen/Dense>

class KalmanFilter
{
public:
  KalmanFilter() = default;
  ~KalmanFilter() = default;

  // init the filter
  void init(double dt);

  void predict();
  void update(const Eigen::VectorXd& z);

  // setters
  void setState(double x, double y);

  // getters
  double getXCovariance() const { return P_.coeff(0, 0); }
  double getYCovariance() const { return P_.coeff(1, 1); }
  double getX() const { return x_[0]; }
  double getY() const { return x_[1]; }

private:
  // dt in seconds
  double dt_;

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transistion matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;
};

#endif // KALMAN_FILTER_H_
