#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x, MatrixXd &P, MatrixXd (*GetF)(float),
                        MatrixXd (*GetH)(), MatrixXd (*GetQ)(float), MatrixXd (*GetR)())
{
    this->x_ = x;
    this->P_ = P;
    this->GetF = GetF;
    this->GetH = GetH;
    this->GetR = GetR;
    this->GetQ = GetQ;
}

void KalmanFilter::Predict(float delta_T)
{
    /**
     * predict the state
     */

    // update state transition matrix and process covariance matrix
    MatrixXd F = GetF(delta_T);
    MatrixXd Q = GetQ(delta_T);

    // predict state
    x_ = F * x_; // + v

    // predict state covariance
    P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilter::Update(const VectorXd &z)
{
    /**
     * update the state by using Kalman Filter equations
     */
    MatrixXd H = GetH();
    MatrixXd R = GetR();
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);

    // measurement update
    VectorXd y = z - H * x_;
    MatrixXd S = H * P_ * H.transpose() + R;
    MatrixXd K = P_ * H.transpose() * S.inverse();

    // new estimate
    x_ = x_ + (K * y);
    P_ = (I - K * H) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, VectorXd (*GetY)(const VectorXd&, const VectorXd&), MatrixXd (*GetHj)(const VectorXd&), MatrixXd (*GetR)())
{
    /**
     * TODO: update the state by using Extended Kalman Filter equations
     */
    MatrixXd Hj = GetHj(x_);                   // calculate jacobian using current state
    MatrixXd R = GetR();                        // overrides member function
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);

    // measurement update
    VectorXd y = GetY(z, x_);                      //y = z-hx, with angle normailzation
    MatrixXd S = Hj * P_ * Hj.transpose() + R;      //use Hj
    MatrixXd K = P_ * Hj.transpose() * S.inverse(); //use Hj

    // new estimate
    x_ = x_ + (K * y);
    P_ = (I - K * Hj) * P_;                         //use Hj
}
