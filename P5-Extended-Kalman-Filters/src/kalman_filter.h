#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
#include <functional>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter
{
public:
    /**
   * Constructor
   */
    KalmanFilter();

    /**
   * Destructor
   */
    virtual ~KalmanFilter();

    /**
   * Init Initializes Kalman filter
   * @param x Initial state
   * @param P Initial state covariance
   * @param GetF Transition matrix
   * @param Geth Measurement matrix
   * @param GetQ Process covariance matrix
   * @param GetR Measurement covariance matrix
   */
    void Init(VectorXd &x, MatrixXd &P,
              MatrixXd (*GetF)(float), MatrixXd (*GetH)(),
              MatrixXd (*GetQ)(float), MatrixXd (*GetR)());

    /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   *  @param delta_T Time between k and k+1 in s
   */
    void Predict(float delta_T);

    /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
    void Update(const Eigen::VectorXd &z);

    /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z      The measurement at k+1
   * @param GetY   Function to call instead of default H matrix to use extended KF.
   * @param GetHj  Return Jacobian matrix of h()
   * @param GetR   Return Measurement covariance matrix for EKF
   */
    void UpdateEKF(const Eigen::VectorXd &z,
                   VectorXd (*GetY)(const VectorXd &, const VectorXd &),
                   MatrixXd (*GetHj)(const VectorXd &),
                   MatrixXd (*GetR)());

    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // returns the state transition matrix
    Eigen::MatrixXd (*GetF)(float);

    // process covariance matrix
    Eigen::MatrixXd (*GetQ)(float);

    // measurement matrix
    Eigen::MatrixXd (*GetH)();

    // measurement covariance matrix
    Eigen::MatrixXd (*GetR)();
};

#endif // KALMAN_FILTER_H_
