#include <tracker/KalmanFilter.h>

#define LOGGING_ON true
#include <logger.hpp>

void KalmanFilter::init(double dt) {
  dt_ = dt;

  // create a 4D state vector
  x_ = Eigen::VectorXd(4);

  // TODO: Initialize the state covariance matrix P
  P_ = Eigen::MatrixXd(4, 4);
  P_ << 1., 0., 0., 0.,
      0., 1., 0., 0.,
      0., 0., 9999., 0.,
      0., 0., 0., 9999.;

  // measurement covariance
  R_ = Eigen::MatrixXd(2, 2);
  R_ << 0.0225, 0.,
      0., 0.0225;

  // measurement matrix
  H_ = Eigen::MatrixXd(2, 4);
  H_ << 1., 0., 0., 0.,
      0., 1., 0., 0.;

  // the transition matrix F
  F_ = Eigen::MatrixXd(4, 4);
  F_ << 1., 0., dt_, 0.,
      0., 1., 0., dt_,
      0., 0., 1., 0.,
      0., 0., 0., 1.;

  // set the acceleration noise components
  double noise_ax_ = 2.;
  double noise_ay_ = 2.;

  double dt_2 = dt_ * dt_;
  double dt_3 = dt_2 * dt_;
  double dt_4 = dt_3 * dt_;

  // set the process noise matrix Q
  Q_ = Eigen::MatrixXd(4, 4);
  Q_ << dt_4 / 4. * noise_ax_, 0., dt_3 / 2. * noise_ax_, 0.,
      0., dt_4 / 4. * noise_ay_, 0., dt_3 / 2. * noise_ay_,
      dt_3 / 2. * noise_ax_, 0., dt_2 * noise_ax_, 0.,
      0., dt_3 / 2. * noise_ay_, 0., dt_2 * noise_ay_;
}

void KalmanFilter::predict()
{
  // U matrix (input variable) doesn't exist in this problem since we are not controlling the system/plant.
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd &z)
{
  // Temporary matrices
  Eigen::VectorXd y = z - H_ * x_;
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

  // New estimate
  x_ = x_ + (K * y);
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
  
  static auto logger = logger::Logger("KF");
  logger << P_;
}

void KalmanFilter::setState(double x, double y)
{
  x_ << x, y, 0., 0.;
}