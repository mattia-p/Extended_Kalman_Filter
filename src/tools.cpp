#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

    // initialize the rmse vector
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

   if (estimations.size() == 0 || estimations.size() != ground_truth.size()){
       cout << "Estimation or ground truth data invalid" << endl;
   }

   for (int i = 0; i < estimations.size(); i++){
       VectorXd residual = estimations[i] - ground_truth[i];
       // multiplication term by term
       residual = residual.array()*residual.array();
       rmse += residual;
   }

   // calculate the mean for each component of the RMSE
   rsme = rme/estimations.size();

   // calculate the squared root
   rmse = rmse.array().sqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}
