#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
   */
   VectorXd y = z - H_ * x_;
   MatrixXd H_tran = H_.transpose();
   MatrixXd S = H_ * P_ * H_tran + R_;
   MatrixXd S_inv = S.inverse();
   MatrixXd k = P_ * H_tran * S_inv;

   // Updated State
   x_ = x_ + (k * y);
   MatrixXd Iden = MatrixXd::Identity(x_.size(), x_.size());
   P_ = (Iden - k * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
   double px = x_(0);
   double py = x_(1);
   double vx = x_(2);
   double vy = x_(3);

   double rho = sqrt(px * px + py * py);
   double theta = atan2(py, px);
   double rho_dot = (px * vx + py * vy) / rho;

   VectorXd H = VectorXd(3);
   H << rho, theta, rho_dot;
   VectorXd y = z - H;
   while ( y(1) > M_PI || y(1) < -M_PI) {
    if( y(1) > M_PI ) {
      y(1) -= M_PI;
    } else {
      y(1) += M_PI;
    }
   }
   
   MatrixXd H_tran = H_.transpose();
   MatrixXd S = H_ * P_ * H_tran + R_;
   MatrixXd S_inv = S.inverse();
   MatrixXd k = P_ * H_tran * S_inv;

   //new estimate
   x_ = x_ + (k * y);
   long x_size = x_.size();
   MatrixXd I = MatrixXd::Identity(x_size, x_size);
   P_ = (I - k * H_) * P_;
}

