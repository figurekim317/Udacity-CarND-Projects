#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() == 0)
    {
        cout << "empty estimation" << endl;
        return rmse;
    }
    else if (estimations.size() != ground_truth.size())
    {
        cout << "size does not match" << endl;
        return rmse;
    }

    // accumulate squared error
    for (unsigned int i = 0; i < estimations.size(); ++i)
    {
        VectorXd residual = estimations[i] - ground_truth[i];
        VectorXd square = residual.array() * residual.array();
        rmse += square;
    }

    // calculate the mean
    rmse /= estimations.size();

    // calculate the squared root
    rmse = rmse.array().sqrt();

    // return the result
    return rmse;
}
