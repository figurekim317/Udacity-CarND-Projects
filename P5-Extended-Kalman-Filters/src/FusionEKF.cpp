#include "FusionEKF.h"
#include <iostream>
#include <cmath>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF()
{
    is_initialized_ = false;
    previous_timestamp_ = 0;

    // initial state
    VectorXd x = VectorXd(4);
    x << 0, 0, 0, 0;

    // initial state covariance
    MatrixXd P = MatrixXd(4, 4);
    P << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;

    ekf_.Init(x, P, CalculateF, CalculateH, CalculateQ, CalculateR);

    previous_timestamp_ = 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
    /**
     * Initialization
     */
    if (!is_initialized_)
    {
        // first measurement
        cout << "EKF: " << endl;

        float px = 0, py = 0;
        // if radar, convert polar to cartesian coordinate
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            float rho = measurement_pack.raw_measurements_[0];
            float pi = measurement_pack.raw_measurements_[1];

            px = rho * cos(pi);
            py = rho * sin(pi);
        }
        // if laser, initialize raw value
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
        {
            px = measurement_pack.raw_measurements_[0];
            py = measurement_pack.raw_measurements_[1];
        }

        // done initializing, no need to predict or update
        ekf_.x_ << px, py, 0, 0;
        ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;

        return;
    }

    /**
     * Prediction
     * Update the state transition matrix F according to the new elapsed time.
     * Time is measured in seconds.
     * Predict() will update F and Q with callback attached when initialized.
     */

    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;
    ekf_.Predict(dt);

    /**
     * Update
     * - Use the sensor type to perform the update step.
     * - Update the state and covariance matrices.
     */
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        float rho = measurement_pack.raw_measurements_[0];
        float pi = measurement_pack.raw_measurements_[1];
        float rho_dot = measurement_pack.raw_measurements_[2];

        // Radar updates
        VectorXd z = VectorXd(3);
        z << rho, pi, rho_dot;
        ekf_.UpdateEKF(z, CalculateRadarY, CalculateRadarJacobian, CalculateRadarR);
    }
    else
    {
        float px = measurement_pack.raw_measurements_[0];
        float py = measurement_pack.raw_measurements_[1];

        // Laser updates
        VectorXd z = VectorXd(2);
        z << px, py;
        ekf_.Update(z);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}

// transition
MatrixXd FusionEKF::CalculateF(float delta_T)
{
    MatrixXd F = MatrixXd(4, 4);
    F << 1, 0, delta_T, 0,
        0, 1, 0, delta_T,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return F;
};

// measurement
MatrixXd FusionEKF::CalculateH()
{
    MatrixXd H = MatrixXd(2, 4);
    H << 1, 0, 0, 0,
        0, 1, 0, 0;

    return H;
};

// process covariance
MatrixXd FusionEKF::CalculateQ(float delta_T)
{
    // measurement acceleration covariance
    float noise_ax = 9;
    float noise_ay = 9;

    MatrixXd Qv = MatrixXd(2, 2);
    Qv << noise_ax, 0,
        0, noise_ay;

    float sq = delta_T * delta_T / 2.0;
    MatrixXd G = MatrixXd(4, 2);
    G << sq, 0,
        0, sq,
        delta_T, 0,
        0, delta_T;

    MatrixXd Q = G * Qv * G.transpose();

    return Q;
};

// measurement covariance
MatrixXd FusionEKF::CalculateR()
{
    MatrixXd R = MatrixXd(2, 2);
    R << 0.0225, 0,
        0, 0.0225;

    return R;
};

// radar y = z - hx
VectorXd FusionEKF::CalculateRadarY(const VectorXd &z, const VectorXd &x_state)
{
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    /**
     * check division by zero
     */
    if (px * px + py * py == 0)
    {
        VectorXd hx = VectorXd(3);
        hx << 0, 0, 0;
        return hx;
    }

    float rho = sqrt(px * px + py * py);
    float pi = atan2(py, px); // value between -pi ~ pi
    float rho_dot = (px * vx + py * vy) / rho;

    VectorXd hx = VectorXd(3);
    hx << rho, pi, rho_dot;

    VectorXd y = z - hx;
    while (y(1) > M_PI)
        y(1) -= 2 * M_PI;
    while (y(1) <= -M_PI)
        y(1) += 2 * M_PI;

    return y;
}

// radar measurement jacobian
MatrixXd FusionEKF::CalculateRadarJacobian(const VectorXd &x_state)
{
    MatrixXd Hj = MatrixXd(3, 4);
    Hj << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // if divided by zero, the measurement is overlaped with the car
    if (px * px + py * py == 0)
    {
        return Hj; //return zero value
    }

    float p_abs = sqrt(px * px + py * py);

    // compute the Jacobian matrix
    // derivative of rho
    Hj(0, 0) = px / p_abs;
    Hj(0, 1) = py / p_abs;

    // derivative of pi
    Hj(1, 0) = -py / (px * px + py * py);
    Hj(1, 1) = px / (px * px + py * py);

    // derivative of rho dot
    Hj(2, 0) = py * (vx * py - vy * px) / pow(p_abs, 3);
    Hj(2, 1) = px * (vy * px - vx * py) / pow(p_abs, 3);
    Hj(2, 2) = px / p_abs;
    Hj(2, 3) = py / p_abs;

    return Hj;
}

// radar measurement covariance
MatrixXd FusionEKF::CalculateRadarR()
{
    //measurement covariance matrix - radar
    //Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
    MatrixXd R = MatrixXd(3, 3);
    R << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

    return R;
}
