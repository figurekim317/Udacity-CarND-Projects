#include "FusionEKF.h"
#include <iostream>
#include <cmath>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

VectorXd CalculateRadarMeasurementH(const VectorXd &);
MatrixXd CalculateRadarMeasurementJacobian(const VectorXd &);

/**
 * Constructor.
 */
FusionEKF::FusionEKF()
{
    is_initialized_ = false;
    previous_timestamp_ = 0;

    // initial state
    VectorXd x = VectorXd(4);

    // initial state covariance
    MatrixXd P = MatrixXd(4, 4);

    // lambda transition
    MatrixXd (*GetF)(float) = [](float delta_T)
    {
        MatrixXd F = MatrixXd();
        F << 1, 0, delta_T, 0,
            0, 1, 0, delta_T,
            0, 0, 1, 0,
            0, 0, 0, 1;

        return F;
    };

    // lambda measurement
    MatrixXd (*GetH)() = []()
    {
        MatrixXd H = MatrixXd(2, 4);
        H << 1, 0, 0, 0,
            0, 1, 0, 0;

        return H;
    };

    // lambda measurement covariance
    MatrixXd (*GetR)() = []()
    {
        MatrixXd R = MatrixXd(2, 2);
        R << 0.0225, 0,
            0, 0.0225;

        return R;
    };

    // lambda process covariance
    MatrixXd (*GetQ)(float) = [](float delta_T)
    {
        MatrixXd Q = MatrixXd(4, 4);
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

        return Q;
    };

    ekf_.Init(x, P, GetF, GetH, GetR, GetQ);

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

        // if radar, convert polar to cartesian coordinate
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            float rho = measurement_pack.raw_measurements_[0];
            float pi = measurement_pack.raw_measurements_[1];

            float px = rho * cos(pi);
            float py = rho * sin(pi);

            ekf_.x_ << px, py, 0, 0;
            ekf_.P_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;
        }
        // if laser, initialize raw value
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
        {
            float px = measurement_pack.raw_measurements_[0];
            float py = measurement_pack.raw_measurements_[1];

            ekf_.x_ << px, py, 0, 0;
            ekf_.P_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;
        }

        // done initializing, no need to predict or update
        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;

        return;
    }

    /**
     * Prediction
     * 
     * Update the state transition matrix F according to the new elapsed time.
     * Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
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
        ekf_.UpdateEKF(z, CalculateRadarMeasurementH, CalculateRadarMeasurementJacobian);
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

VectorXd CalculateRadarMeasurementH(const VectorXd &x_state)
{
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float rho = sqrt(px * px + py * py);
    /**
     * TODO: check division by zero
     */
    float pi = atan2(px, py); // value between -pi ~ pi
    float rho_dot = (px * vx + py * vy) / rho;

    VectorXd hx = VectorXd(3);
    hx << rho, pi, rho_dot;

    return hx;
}

MatrixXd CalculateRadarMeasurementJacobian(const VectorXd &x_state)
{
    MatrixXd Hj(3, 4);
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // TODO: YOUR CODE HERE

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
