#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
	  x_ = x_in; // Object state
	  P_ = P_in; // Object covariance matrix
	  F_ = F_in; // State transiction matrix
	  H_ = H_in; // Measurement matrix
	  R_ = R_in; // Measurement covariance matrix
	  Q_ = Q_in; // Process covariance matrix
}

void KalmanFilter::Predict() {
  /**************************************
   * TODO: predict the state
   *
   * Same for linear and extended KF
   * ***********************************/
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */

    VectorXd y = z - H_*x_;

    CommonUpdate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

    float px = x_[0];
    float py = x_[1];
    float vx = x_[2];
    float vy = x_[3];


    double rho     = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    double phi     = atan2(x_(1), x_(0));

    double corrected_rho = rho;

    if (corrected_rho == 0)
        corrected_rho = 0.0001;

    double rho_dot = ((px * vx) + (py * vy)) / corrected_rho;

    VectorXd x_pred(3);
    x_pred << rho, phi, rho_dot;

    VectorXd y = z - x_pred;


    CommonUpdate(y);
}


void KalmanFilter::CommonUpdate(const VectorXd &y){

  MatrixXd Ht  = H_.transpose();
  MatrixXd S   = H_ * P_ * Ht + R_;
  MatrixXd Si  = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K   = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
