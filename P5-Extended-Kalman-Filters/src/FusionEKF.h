#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF
{
public:
    /**
     * Constructor.
     */
    FusionEKF();

    /**
     * Destructor.
     */
    virtual ~FusionEKF();

    /**
     * Run the whole flow of the Kalman Filter from here.
     */
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);

    /**
     * Kalman Filter update and prediction math lives in here.
     */
    KalmanFilter ekf_;

private:
    // check whether the tracking toolbox was initialized or not (first measurement)
    bool is_initialized_;

    // previous timestamp
    long long previous_timestamp_;

    static MatrixXd CalculateF(float);      // state transition matrix
    static MatrixXd CalculateH();           // measurement matrix
    static MatrixXd CalculateQ(float);      // process covariance matrix
    static MatrixXd CalculateR();           // measurement covariance matrix
    static VectorXd CalculateRadarY(const VectorXd &, const VectorXd &); // measurement y = z - hx
    static MatrixXd CalculateRadarJacobian(const VectorXd &);            // jacobian Hj
    static MatrixXd CalculateRadarR();                                   // radar measurement covariance

    // tool object used to compute Jacobian and RMSE
    Tools tools;
};

#endif // FusionEKF_H_
