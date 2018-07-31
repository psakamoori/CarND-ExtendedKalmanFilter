#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if( estimations.size() == 0) {
     cout << "ERROR: Estimate Vector is Empty" << endl;
     return rmse;
  }
 
  if( ground_truth.size() == 0) {
    cout << "ERROR: Ground truth Vector is Epmty" << endl;
    return rmse;
  }

  unsigned int no_of_estimates = estimations.size();
  if( no_of_estimates != ground_truth.size()) {
    cout << "ERROR: No of Estaimtes not equal to GT vectors size" << endl;
    return rmse;
  }

  for(unsigned int i = 0; i < no_of_estimates; i++) {
   VectorXd diff = estimations[i] - ground_truth[i];
   diff = diff.array()*diff.array();
   rmse += diff;
  }

  rmse = rmse / no_of_estimates;
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);

  if( x_state.size() != 4) {
    cout << "ERROR: State Vector size less than 4." << endl;
    return Hj;
  }

  // State Parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  double d1 = px * px + py * py;
  double d2 = sqrt(d1);
  double d3 = (d1 * d2);

  // Checking division by zero condition
  if(fabs(d1) < 0.0001) {
    cout << "ERROR: Division by Zero" << endl;
    return Hj;
  }

  // Compute Jacobian Matrix
  Hj << (px/d2), (py/d2), 0, 0,
        -(py/d1), (px/d1), 0, 0,
        py*(vx * py - vy * px)/d3, px*(px * vy - py * vx)/d3, px/d2, py/d2;

  return Hj;
}
