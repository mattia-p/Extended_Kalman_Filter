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
    // jacobian
    Hj = MatrixXd(3,4);

    float px = x_state[0];
    float py = x_state[1];
    float vx = x_state[2];
    float vy = x_state[3];

    // precompute some variables to avoid repetitions
    float c1 = px * px + py * py;
    float c2 = sqrt(c1);
    float c3 = (c1*c2);

    // check if division by zero
    if (fabs(c1) < 0.0001){
        cout << "Error in computing jacobian - division by zero" << endl;
        return
    }


    Hj << px / c2, py/c2, 0, 0,
          -py/c1, px/c1, 0, 0,
          py*(vx*py - vy*px)/c2, px*(vy*px - vx*py)/c3, px/c2, py/c2;

    return Hj;
}
